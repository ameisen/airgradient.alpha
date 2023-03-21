#include "AirGradient.h"
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
//#include <WiFiManager.h>

#include "SGP30.h"
#include <U8g2lib.h>

#include <type_traits>
#include <limits>
#include <array>
#include <experimental/array>

#define VERTICAL 0
#define HORIZONTAL 1
#define SCREEN_ORIENTATION HORIZONTAL

namespace {
  static const String na_string = "N/A";

  namespace config {
    namespace wifi {
      static constexpr const bool enable = true;
      static constexpr const bool server_mode = true;

      static constexpr const char ssid[] = "Stackheap";
      static constexpr const char password[] = "d34db33f";

      static constexpr const char endpoint[] = "http://hw.airgradient.com/";
    } // namespace wifi

    // set to true if Fahrenheit is preferred
    static constexpr const bool fahrenheit = true;
  } // namespace config

  static AirGradient ag = {};
  static SGP30 SGP;

  // Display bottom right
  static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/U8X8_PIN_NONE);

  namespace state {
    static unsigned long current_timestamp = 0ul;

    template<int interval_value>
    struct device_state {
      static constexpr const int interval = interval_value;
      bool started = false;
      unsigned long last_time = 0ul;

      device_state() = default;
      device_state(bool _started) {
        started = _started;
      }

      bool update_interval() __restrict {
        if (state::current_timestamp - last_time >= interval) {
          last_time += interval;
          return true;
        }
        return false;
      }
    };

    template <typename T, int interval_value>
    struct sensor_state : device_state<interval_value> {
      T value = std::numeric_limits<T>::min();

      sensor_state() = default;
      sensor_state(bool _started) : device_state<interval_value>(_started) {
      }

      String to_string() const __restrict {
        if constexpr (std::is_fundamental_v<T>) {
          if (value == std::numeric_limits<T>::min()) {
            return na_string;
          }
          else {
            if constexpr (std::is_floating_point_v<T>) {
              if (!std::isfinite(value)) {
                return na_string;
              }
              return String(value, 1);
            }
            else {
              return String(value);
            }
          }
        }
        else {
          return {};
        }
      }
    };

    template <typename T, int interval_value>
    struct tvoc_state : sensor_state<T, interval_value> {
      float h2 = std::numeric_limits<float>::min();
      float ethanol = std::numeric_limits<float>::min();

      tvoc_state() = default;
      tvoc_state(bool _started) : sensor_state<T, interval_value>(_started) {
      }
    };

    namespace wifi {
      static bool is_offline = false;
      static bool server_open = false;
      static util::lazy<WiFiServer> server = {};

      static IPAddress address = {};
      static device_state<20'000> device = { false };
    }

    static device_state<5'000> oled = { true };
    static device_state<10'000> server = { true };

    static tvoc_state<int, 1'000> tvoc = {};
    static sensor_state<int, 5'000> co2 = { true };
    static sensor_state<int, 5'000> pm25 = { true };
    static sensor_state<float, 2'500> temperature = { true };
    static sensor_state<int, 2'500> humidity = { true };
  }

  namespace screen_horizontal {
    static const uint8_t * const __restrict font = u8g2_font_t0_14_tf;

    static constexpr const int width = 128 + 8;
    static constexpr const int height = 64;

    static constexpr const int font_width = 8;
    static constexpr const int font_height = 13;

    static constexpr const int padding = 0;
    static constexpr const int height_offset = 10;

    static constexpr const int columns = (width - (padding * 2)) / font_width;
    static constexpr const int rows = (height - (padding * 2)) / font_height;
  }

  namespace screen_vertical {
    static const uint8_t * const __restrict font = screen_horizontal::font;

    static constexpr const int width = 64;
    static constexpr const int height = 128;

    static constexpr const int font_width = screen_horizontal::font_height;
    static constexpr const int font_height = screen_horizontal::font_width;

    static constexpr const int padding = screen_horizontal::padding;
    static constexpr const int height_offset = font_height - padding - 1;

    static constexpr const int columns = (width - (padding * 2)) / font_width;
    static constexpr const int rows = (height - (padding * 2)) / font_height;
  }

  namespace screen {
  #if SCREEN_ORIENTATION == HORIZONTAL
    using namespace screen_horizontal;
  #else
    using namespace screen_vertical;
  #endif
  }

  template <typename... LinesArg>
  static String equalSpace(const LinesArg&... lines) {
    const int length = screen::columns;

    int numLines = sizeof...(lines);
    int numGaps = numLines + 1;
    int totalLength = (... + lines.length());

    int remaining = length - totalLength;
    if (remaining <= 0) {
      String concat = (... + lines);
      return concat;
    }
    else {
      String concat;

      int spacing = remaining / numGaps;
      int spacingRemaining = remaining - (spacing * numGaps);

      const auto append = [&](const String &str) {
        int space = spacing;
        if (spacingRemaining > 0) {
          ++space;
          --spacingRemaining;
        }

        for (; space > 0; --space) {
          concat += ' ';
        }

        concat += str;
      };

      (append(lines), ...);
      return concat;
    }
  }

  static String centerText(const String &str, int length = screen::columns, bool padOut = false) {
    int strLen = int(str.length());
    int remaining = (length - strLen) / 2;
    if (remaining < 0) {
      remaining = 0;
    }
    int padRemaining = length - (remaining + strLen);
    if (padRemaining > remaining) {
      --padRemaining;
      ++remaining;
    }
    if (remaining == 0 && (!padOut || padRemaining <= 0)) {
      return str;
    }

    String out;
    for (; remaining > 0; --remaining) {
      out += ' ';
    }
    out += str;

    if (padOut) {
      for (; padRemaining > 0; --padRemaining) {
        out += ' ';
      }
    }

    return out;
  }

  static int getRowOffset(int row) {
    return screen::height_offset + (screen::font_height * row);
  }

  template <typename... LinesArg>
  static void updateOLED(const LinesArg&... lines) {
    int row = 0;

    const auto DrawLine = [&row](const String &str) {
      u8g2.drawStr(screen::padding, getRowOffset(row), str.c_str());

      ++row;
    };

    u8g2.clearBuffer();
    u8g2.enableUTF8Print();
    u8g2.setFont(screen::font);
    (DrawLine(lines), ...);
    u8g2.sendBuffer();
  }

  template <size_t N>
  static void updateOLED(const std::array<String, N> &lines) {
    int row = 0;

    const auto DrawLine = [&row](const String &str) {
      u8g2.drawStr(screen::padding, getRowOffset(row), str.c_str());

      ++row;
    };

    u8g2.clearBuffer();
    u8g2.enableUTF8Print();
    u8g2.setFont(screen::font);
    for (const auto &line : lines) {
      DrawLine(line);
    }
    u8g2.sendBuffer();
  }

  static bool updateTVOC() {
    static constexpr const int retry_count = 5;

    if _unlikely(!state::tvoc.started || !SGP.isConnected()) [[unlikely]] {
      return false;
    }

    if (!state::tvoc.update_interval()) {
      return false;
    }

    int retries = retry_count;
    bool error;
    do {
      error = false;
      if (SGP.measure(true)) {
        const float uninitialized_value = 67130187776.0f;

        state::tvoc.value = SGP.getTVOC();
        error = state::tvoc.value == std::numeric_limits<decltype(state::tvoc.value)>::max();
        state::tvoc.h2 = SGP.getH2();
        // This seems to be what gets returned for an uninitialized value
        if _unlikely(state::tvoc.h2 == uninitialized_value || !std::isfinite(state::tvoc.h2)) [[unlikely]] {
          state::tvoc.h2 = std::nanf("");
          error = true;
        }
        state::tvoc.ethanol = SGP.getEthanol();
        if _unlikely(state::tvoc.ethanol == uninitialized_value || !std::isfinite(state::tvoc.ethanol)) [[unlikely]] {
          state::tvoc.ethanol = std::nanf("");
          error = true;
        }
        //Serial.println(String("tvoc: ") + state::tvoc.to_string());
        //Serial.println(String("h2: ") + String(state::tvoc.h2));
        //Serial.println(String("eth: ") + String(state::tvoc.ethanol));
      }
    } while (_unlikely(error) && retries--);

    if _unlikely(error) [[unlikely]] {
      ag.debugln("TVOC error");
    }

    return true;
  }

  static bool updateCo2() {
    static constexpr const int retry_count = 5;

    if (!state::co2.update_interval()) {
      return false;
    }

    int retries = retry_count;
    int value;
    do {
      value = ag.getCO2_Raw();
    } while (_unlikely(value < 0) && retries--);

    if _unlikely(value < 0) [[unlikely]] {
      ag.debugfln("CO2 error: %d", value);
      value = std::numeric_limits<int>::min();
    }
    state::co2.value = value;
    //Serial.println(String("co2: ") + state::co2.to_string());

    return true;
  }

  static bool updatePm25() {
    static constexpr const int retry_count = 5;

    if (!state::pm25.update_interval()) {
      return false;
    }

    int retries = retry_count;
    int value;
    do {
      value = ag.getPM2_Raw();
    } while(_unlikely(value < 0) && retries--);

    if _unlikely(value < 0) [[unlikely]] {
      ag.debugfln("PM25 error: %d", value);
      value = std::numeric_limits<int>::min();
    }
    state::pm25.value = value;
   // Serial.println(String("pm25: ") + state::pm25.to_string());

    return true;
  }

  static bool updateTempHum() {
    static constexpr const int retry_count = 5;

    using namespace zephyr;
    using error_code = zephyr::temperature_humidity::error_code;

    if (!state::temperature.update_interval()) {
      return false;
    }

    int retries = retry_count;
    temperature_humidity::data result;
    do {
      result = temperature_humidity::read();
    } while (_unlikely(result.error != error_code::no_error) && retries--);

    state::temperature.value = result.temperature;
    state::humidity.value = result.relative_humidity;

    if _unlikely(result.error != error_code::no_error)  [[unlikely]] {
      ag.debugfln("TempHum error: %d", result.error);
    }
    else if (state::tvoc.started && SGP.isConnected()) {
      SGP.setRelHumidity(result.temperature, result.relative_humidity);
    }

    //Serial.println(String("temperature: ") + state::temperature.to_string());
    //Serial.println(String("humidity: ") + state::humidity.to_string());

    return true;
  }

  struct MeasurePair {
    String Key;
    String Value;
  };

  // Calculate PM2.5 US AQI
  static int getAQI(int pm02) {
    if (pm02 == 0)
      return 0;
    else if (pm02 <= 12)
      return ((50 - 0) / (12.0 - .0) * (pm02 - .0) + 0) + 0.5;
    else if (pm02 <= 35)
      return ((100 - 50) / (35.4 - 12.0) * (pm02 - 12.0) + 50) + 0.5;
    else if (pm02 <= 55)
      return ((150 - 100) / (55.4 - 35.4) * (pm02 - 35.4) + 100) + 0.5;
    else if (pm02 <= 150)
      return ((200 - 150) / (150.4 - 55.4) * (pm02 - 55.4) + 150) + 0.5;
    else if (pm02 <= 250)
      return ((300 - 200) / (250.4 - 150.4) * (pm02 - 150.4) + 200) + 0.5;
    else if (pm02 <= 350)
      return ((400 - 300) / (350.4 - 250.4) * (pm02 - 250.4) + 300) + 0.5;
    else if (pm02 <= 500)
      return ((500 - 400) / (500.4 - 350.4) * (pm02 - 350.4) + 400) + 0.5;
    else
      return 501;
  }

  static int iround(float value) {
    return (int)std::roundf(value);
  }

  template <typename T>
  static String getTVOCValue(T value, bool roundValue = false) {
    if (!state::tvoc.started || !SGP.isConnected()) {
      return na_string;
    }

    if (value < 0 || value == std::numeric_limits<T>::min() || value == std::numeric_limits<T>::max()) {
      return na_string;
    }

    if (std::is_same_v<T, float> && roundValue) {
      if (value > std::numeric_limits<T>::max() || value < std::numeric_limits<T>::min()) {
        return na_string;
      }

      return String(iround(value));
    }

    return String(value);
  }

  static String getAQIString(int pm02) {
    int aqi = getAQI(pm02);
    if _unlikely(aqi < 0) [[unlikely]] {
      return na_string;
    }
    return String(aqi);
  }

  static bool updateOLED() {
    if (!state::oled.update_interval()) {
      return false;
    }

    auto temperature = state::temperature.value;
    if (config::fahrenheit) {
      temperature = (temperature * 1.8f) + 32.0f;
    }

    auto pairs = std::experimental::make_array(
        MeasurePair{"PM", state::pm25.to_string()},
        MeasurePair{"CO2", state::co2.to_string()},
        MeasurePair{"AQI", getAQIString(state::pm25.value)},
        MeasurePair{"VOC", getTVOCValue(state::tvoc.value)},
        MeasurePair{"H2", getTVOCValue(state::tvoc.h2 * 1000.0f, true)},
        MeasurePair{"ETH", getTVOCValue(state::tvoc.ethanol * 1000.0f, true)}
    );

    std::array<String, (pairs.size() / 2) + 2> lines;

    {
      const String temperatureString = String(temperature, 1) + (config::fahrenheit ? " \xB0""F" : " \xB0""C");
      const String humidityString = state::humidity.to_string() + "%";

      lines[0] = equalSpace(temperatureString, humidityString);
      //lines[0] = centerText(temperatureString, screen::columns / 2, true) + centerText(humidityString, screen::columns / 2, true);
    }

    {
      int maxKeyLen = 0;
      for (const auto &pair : pairs) {
        int len = pair.Key.length();
        if (maxKeyLen < len) {
          maxKeyLen = len;
        }
      }

      int maxLen = screen::columns / 2;
      int maxLenRemainder = screen::columns - (maxLen + maxLen);

      for (uint i = 0; i < pairs.size() / 2; ++i) {
        const auto &pair0 = pairs[i * 2 + 0];
        const auto &pair1 = pairs[i * 2 + 1];

        const auto push_pair = [maxKeyLen, maxLen](const MeasurePair &p, int addLen = 0) -> String {
          int remainingLen = maxLen + addLen;

          int keyLen = p.Key.length();
          int valueLen = p.Value.length();

          String line = p.Key;
          line += ':';
          remainingLen -= keyLen + 1;

          if (remainingLen >= 0 && remainingLen > valueLen) {
            int remaining = remainingLen - valueLen;
            for (; remaining > 0; --remaining) {
              line += ' ';
            }
          }

          line += p.Value;

          return line;
        };

        int line = i + 1;

        lines[line] = push_pair(pair0);
        lines[line] += ' ';
        lines[line] += push_pair(pair1, maxLenRemainder);
      }

      lines[lines.size() - 1] = centerText(state::wifi::device.started ? state::wifi::address.toString() : "offline");
    }

    updateOLED(lines);

    return true;
  }

  static void sendToServer() {
    if (config::wifi::server_mode) {
      return;
    }

    if (!state::server.update_interval()) {
      return;
    }

    if (WiFi.status() == WL_CONNECTED) {
      String payload =
          "{\"wifi\":" + String(WiFi.RSSI()) + ", \"rco2\":" + String(state::co2.value) +
          ", \"pm02\":" + String(state::pm25.value) + ", \"tvoc\":" + String(state::tvoc.value) +
          ", \"atmp\":" + String(state::temperature.value) + ", \"rhum\":" + String(state::humidity.value) + "}";

      Serial.println(payload);
      String POSTURL = String(config::wifi::endpoint) +
                        "sensors/airgradient:" + String(ESP.getChipId(), HEX) +
                        "/measures";
      Serial.println(POSTURL);
      WiFiClient client;
      HTTPClient http;
      http.begin(client, POSTURL);
      http.addHeader("content-type", "application/json");
      int httpCode = http.POST(payload);
      String response = http.getString();
      Serial.println(httpCode);
      Serial.println(response);
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
  }

  static void updateServer() {
    if (!config::wifi::server_mode) {
      return;
    }

    if (WiFi.status() != WL_CONNECTED) {
      return;
    }

    if (!state::wifi::server) {
      return;
    }

    WiFiClient client = state::wifi::server->accept();
    if (!client) {
      return;
    }
    std::array<char, 4> lastClientData = { 0, 0, 0, 0 };

    const auto pushLastClientData = [&lastClientData] (char c) {
      for (uint i = 0; i < lastClientData.size() - 1; ++i) {
        lastClientData[i] = lastClientData[i + 1];
      }
      lastClientData[lastClientData.size() - 1] = c;
    };

    while (client.connected() && client.available()) {
      char c = client.read();
      pushLastClientData(c);
      //Serial.write(c);

      if (lastClientData[0] == '\r' && lastClientData[1] == '\n' && lastClientData[2] == '\r' && lastClientData[3] == '\n') {
        Serial.println(F("Sending response"));

        client.print(
          "HTTP/1.1 200 OK\r\n"
          "Content-Type: text/html\r\n"
          "Connection: close\r\n"  // the connection will be closed after completion of the response
          "Refresh: 20\r\n"        // refresh the page automatically every 20 sec
          "\r\n");
        client.print(F("<!DOCTYPE HTML>\r\n"));
        client.print(F("<html>\r\n"));

        client.print(F("<h1>"));
        client.print(state::wifi::address.toString());
        client.print(F("</h1>\r\n"));

        auto temperature = state::temperature.value;
        if (config::fahrenheit) {
          temperature = (temperature * 1.8f) + 32.0f;
        }

        const String temperatureString = String(temperature, 1) + (config::fahrenheit ? " \xB0""F" : " \xB0""C");
        const String humidityString = state::humidity.to_string() + "%";

        client.print(F("<h3>T:   "));
        client.print(temperatureString);
        client.print(F("</h3>\r\n"));

        client.print(F("<h3>H:   "));
        client.print(humidityString);
        client.print(F("</h3>\r\n"));

        client.print(F("<h3>PM:  "));
        client.print(state::pm25.to_string());
        client.print(F("</h3>\r\n"));

        client.print(F("<h3>CO2: "));
        client.print(state::co2.to_string());
        client.print(F("</h3>\r\n"));

        client.print(F("<h3>AQI: "));
        client.print(getAQIString(state::pm25.value));
        client.print(F("</h3>\r\n"));

        client.print(F("<h3>VOC: "));
        client.print(getTVOCValue(state::tvoc.value));
        client.print(F("</h3>\r\n"));

        client.print(F("<h3>H2:  "));
        client.print(getTVOCValue(state::tvoc.h2 * 1000.0f, true));
        client.print(F("</h3>\r\n"));

        client.print(F("<h3>ETH: "));
        client.print(getTVOCValue(state::tvoc.ethanol * 1000.0f, true));
        client.print(F("</h3>\r\n"));

        client.print(F("</html>\r\n"));
        break;
      }
    }

    client.stop();
  }

  // Wifi Manager
  static void startWifi() {
    auto status = WiFi.begin(config::wifi::ssid, config::wifi::password);

    if (status == WL_CONNECT_FAILED) {
      state::wifi::is_offline = true;
      return;
    }
  }

  static bool checkWifi() {
    if (!config::wifi::enable) {
      return false;
    }

    if (state::wifi::is_offline) {
      return false;
    }

    if (!state::wifi::device.update_interval()) {
      return false;
    }

    switch (WiFi.status()) {
      case WL_CONNECTED:
        state::wifi::is_offline = false;
        state::wifi::device.started = true;
        state::wifi::address = WiFi.localIP();

        if (state::wifi::server && !state::wifi::server_open) {
          state::wifi::server->begin();
          state::wifi::server_open = true;
        }
        break;
      case WL_CONNECT_FAILED:
      case WL_NO_SHIELD:
        if (!state::wifi::device.started) {
          state::wifi::is_offline = true;
        }
        [[fallthrough]];
      case WL_CONNECTION_LOST:
      case WL_DISCONNECTED:
      case WL_NO_SSID_AVAIL:
      case WL_WRONG_PASSWORD:
        if (state::wifi::server && state::wifi::server_open) {
          state::wifi::server->close();
          state::wifi::server_open = false;
        }
        break;
      default:
        break;
    }

    return true;
  }
}

void setup() {
  Serial.begin(AirGradient::baud_rate);

  SGP.GenericReset();

  u8g2.begin();
  updateOLED();

  updateOLED(String{}, centerText("Initializing"), centerText("sensors."));

  state::tvoc.started = SGP.begin();

  ag.CO2_Init();
  ag.PMS_Init();
  zephyr::temperature_humidity::initialize(0x44);

  if (config::wifi::server_mode) {
    state::wifi::server.initialize(80);
  }

  if (config::wifi::enable) {
    startWifi();
  }

  bool co2_init = ag.CO2_InitJoin(false);
  bool pms_init = ag.PMS_InitJoin(false);
  if (co2_init || pms_init) {
    static constexpr const int warmup_delay = std::max(AirGradient::CO2_WarmUp, AirGradient::PMS_WarmUp);
    Serial.println("Sensors Successfully Initialized. Heating up.");
    delay(warmup_delay);
  }
}

void loop() {
  state::current_timestamp = millis();
  bool update = false;
  update = checkWifi() || update;
  update = updateTVOC() || update;
  update = updateCo2() || update;
  update = updatePm25() || update;
  update = updateTempHum() || update;
  if (update) {
    updateOLED();
    sendToServer();
  }
  updateServer();
}
