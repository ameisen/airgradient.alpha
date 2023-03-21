#include "AirGradient.h"
#include "Arduino.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>
#include <bit>

namespace zephyr {
  namespace {
    class static_class {
      static_class() = delete;
      static_class(const static_class &) = delete;
      static_class(static_class &&) = delete;
    };

    namespace debug {
      static bool message_enabled = true;

      template <typename T>
      static inline void println(const T &value) {
        if _likely(!message_enabled) [[likely]] {
          return;
        }

        Serial.println(value);
      }

      template <typename T>
      static inline void print(const T &value) {
        if _likely(!message_enabled) [[likely]] {
          return;
        }

        Serial.print(value);
      }

      template <typename Tf, typename... Tt>
      static inline void printfln(const Tf &format, Tt&&... values) {
        if _likely(!message_enabled) [[likely]] {
          return;
        }

        Serial.printf(format, std::forward<Tt>(values)...);
        Serial.println();
      }

      template <typename Tf, typename... Tt>
      static inline void printf(const Tf &format, Tt&&... values) {
        if _likely(!message_enabled) [[likely]] {
          return;
        }

        Serial.printf(format, std::forward<Tt>(values)...);
      }
    }

    template <typename T>
    static constexpr std::array<uint8, sizeof(T)> as_byte_array(T &&data) {
      return *(std::array<uint8, sizeof(T)> *)&data;
    }

    template <typename T>
    static constexpr std::array<uint8, sizeof(T)> as_byte_array_swapped(T &&data) {
      auto result = *(std::array<uint8, sizeof(T)> *)&data;

      for (uint i = 0; i < result.size() / 2; ++i) {
        auto temp = result[i];
        uint end_offset = (result.size() - 1) - i;
        result[i] = result[end_offset];
        result[end_offset] = temp;
      }

      return result;
    }

    namespace crc {
      namespace i2c {
        static constexpr uint8 get(const uint8 * __restrict data, uint size) {
          constexpr const uint8 polynomial = 0x31_u8;

          uint8 crc = 0xFF_u8;

          for (uint i = 0; i < size; ++i) {
            crc ^= data[i];
            for (int bit = 8; bit > 0; --bit) {
              const uint8 shifted_crc = crc << 1;
              crc = (crc & 0x80_u8) ?
                shifted_crc ^ polynomial :
                shifted_crc;
            }
          }

          return crc;
        }

        template <uint N>
        static constexpr uint8 get(const uint8 * __restrict data) {
          constexpr const uint8 polynomial = 0x31_u8;

          uint8 crc = 0xFF_u8;

          for (uint i = 0; i < N; ++i) {
            crc ^= data[i];
            for (int bit = 8; bit > 0; --bit) {
              const uint8 shifted_crc = crc << 1;
              crc = (crc & 0x80_u8) ?
                shifted_crc ^ polynomial :
                shifted_crc;
            }
          }

          return crc;
        }

        static constexpr uint8 get(const uint8 * __restrict data) {
          return get<2>(data);
        }

        static constexpr uint8 get(array<uint8, 2> data) {
          return get<2>(data.data());
        }
      }

      namespace modbus {
        namespace impl {
          static constexpr __attribute__((always_inline)) uint16 accumulate(uint16 checksum, uint8 byte) {
            checksum ^= uint16(byte);

            for (int bit = 8; bit != 0; --bit) {
              auto bit_set = checksum & 0x1;
              checksum >>= 1;
              if (bit_set != 0) {
                checksum ^= 0xA001;
              }
            }

            return checksum;
          }
        }

        static constexpr uint16 get(const uint8 * __restrict data, uint size) {
          uint16 checksum = 0xFFFFu;

          for (uint offset = 0; offset < size; ++offset) {
            checksum = impl::accumulate(checksum, data[offset]);
          }

          return checksum;
        }

        template <typename T>
        static constexpr uint16 get(const T & __restrict data) {
          return get((uint8 * __restrict)&data, sizeof(T));
        }

        struct accumulator final {
          uint16 value = 0xFFFFu;

          void accumulate(uint8 byte) {
            value = impl::accumulate(value, byte);
          }

          template <typename T>
          void accumulate(const T * __restrict data, uint size) {
            for (uint offset = 0; offset < size; ++offset) {
              value = impl::accumulate(value, ((const uint8 * __restrict)data)[offset]);
            }
          }

          template <typename T>
          void accumulate(const T & __restrict data) {
            accumulate((const uint8 *)&data, sizeof(T));
          }

          template <typename T>
          void accumulate_swapped(const T * __restrict data, uint size) {
            for (uint offset = 0; offset < size; ++offset) {
              value = impl::accumulate(value, ((const uint8 * __restrict)data)[size - 1 - offset]);
            }
          }

          template <typename T>
          void accumulate_swapped(const T & __restrict data) {
            accumulate_swapped(&data, sizeof(T));
          }
        };
      }
    }
  }
}

AirGradient::AirGradient(bool displayMsg, int baudRate) {
  _debugMsg = displayMsg;
  Wire.begin();
  if (!Serial) {
    Serial.begin(baudRate);
  }
  else {
    baudRate = Serial.baudRate();
  }
  debugln("AirGradient Library instantiated successfully.");
  debugfln("Operating at %d baud", baud_rate);
}

namespace {
  static inline void yield_delay(int milliseconds) {
    (void)milliseconds;
    // Do... something?
  }
}

namespace {
  namespace io {
    template <typename T, typename TStream> requires std::derived_from<TStream, Stream>
    static util::optional<T> optional_read(TStream &stream) {
      if constexpr (sizeof(T) == 1) {
        int result = stream.read();
        if (result == -1) {
          return {};
        }
        return { T(result) };
      }
      else {
        T result;
        if (sizeof(result) != stream.readBytes((uint8*)&result, sizeof(result))) {
          return {};
        }
        return { result };
      }
    }
  }
}

#pragma region PMS5003 PM2

namespace {
  namespace pms5003 {
    static constexpr const int baud_rate = 9'600;

    static constexpr const array<uint8, 2> host_prefix = { 0x42, 0x4D };

    static array<uint8, 12> payload{};
    //static AirGradient::pm_raw_data *raw_data = nullptr;
  }
}

void AirGradient::PMS_Init() {
  PMS_Init(D5, D6);
}
void AirGradient::PMS_Init(int rx_pin, int tx_pin) {
  PMS_Init(rx_pin, tx_pin, pms5003::baud_rate);
}
void AirGradient::PMS_Init(int rx_pin, int tx_pin, int baudRate) {
  debugln("Initializing PMS...");

  auto &stream = _SoftSerial_PMS.initialize(rx_pin, tx_pin);
  stream.begin(baudRate);
}

bool AirGradient::PMS_InitJoin(bool delay) {
  if _unlikely(getPM2_Raw() < 0) [[unlikely]] {
    debugln("PMS Sensor Failed to Initialize ");
    return false;
  }
  else {
    if (delay) {
      Serial.println("PMS Successfully Initialized. Heating up for 10s.");
      yield_delay(10000);
    }
    else {
      Serial.println("PMS Successfully Initialized.");
    }
    return true;
  }
}

int AirGradient::getPM2() {
  auto raw_value = getPM2_Raw();
  if _unlikely(raw_value < 0) [[unlikely]] {
    raw_value = -1;
  }
  return raw_value;
}

static int lastpm02 = -1;

int AirGradient::getPM2_Raw() {
  requestRead();

  pm_raw_data data;
  if (readUntil(data)) {
    lastpm02 = data.PM_AE_UG_2_5;
  }

  if (_PMSstatus != STATUS_OK) {
    debugfln("PMS Fetch failed: %d", _PMSstatus);
  }

  return lastpm02;
}

// Standby mode. For low power consumption and prolong the life of the sensor.
void AirGradient::sleep() {
  static constexpr const uint8_t command[] = {0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};
  _SoftSerial_PMS->write(command, sizeof(command));
}

// Operating mode. Stable data should be got at least 30 seconds after the
// sensor wakeup from the sleep mode because of the fan's performance.
void AirGradient::wakeUp() {
  static constexpr const uint8_t command[] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
  _SoftSerial_PMS->write(command, sizeof(command));
}

// Active mode. Default mode after power up. In this mode sensor would send
// serial data to the host automatically.
void AirGradient::activeMode() {
  static constexpr const uint8_t command[] = {0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71};
  _SoftSerial_PMS->write(command, sizeof(command));
  _mode = MODE_ACTIVE;
}

// Passive mode. In this mode sensor would send serial data to the host only for
// request.
void AirGradient::passiveMode() {
  static constexpr const uint8_t command[] = {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70};
  _SoftSerial_PMS->write(command, sizeof(command));
  _mode = MODE_PASSIVE;
}

// Request read in Passive Mode.
void AirGradient::requestRead() {
  if (_mode != MODE_PASSIVE) {
    return;
  }

  static constexpr const uint8_t command[] = {0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71};
  _SoftSerial_PMS->write(command, sizeof(command));
}

// Non-blocking function for parse response.
bool AirGradient::read_PMS(pm_raw_data & __restrict data) {
  //pms5003::raw_data = &data;
  loop(data);

  return _PMSstatus == STATUS_OK;
}

// Blocking function for parse response. Default timeout is 1s.
bool AirGradient::readUntil(pm_raw_data & __restrict data, uint16_t timeout) {
  //pms5003::raw_data = &data;

  time_ms start = millis();
  do {
    loop(data);
    if (_PMSstatus == STATUS_OK) {
      return true;
    }
  } while (millis() - start < timeout);

  return _PMSstatus == STATUS_OK;
}

void AirGradient::loop(pm_raw_data & __restrict data) {
  _PMSstatus = STATUS_WAITING;

  auto &stream = *_SoftSerial_PMS;

  if (!stream.available()) {
    return;
  }

  util::optional<uint8> ch_opt;
  if (!(ch_opt = io::optional_read<uint8>(stream))) {
    debugln("PMS Loop failed to read checksum");
    return;
  }
  uint8 ch = *ch_opt;

  auto payload = pms5003::payload;
  //array<uint8, 12> payload{};
  //AirGradient::pm_raw_data& data = *pms5003::raw_data;

  const auto inner_func = [&]() {
    switch (_index) {
    case 0:
      if (ch != pms5003::host_prefix[0]) {
        return;
      }
      _calculatedChecksum = ch;
      break;

    case 1:
      if (ch != pms5003::host_prefix[1]) {
        _index = 0;
        return;
      }
      _calculatedChecksum += ch;
      break;

    case 2:
      _calculatedChecksum += ch;
      _frameLen = ch << 8;
      break;

    case 3:
      _frameLen |= ch;
      // Unsupported sensor, different frame length, transmission error e.t.c.
      if (_frameLen != 2 * 9 + 2 && _frameLen != 2 * 13 + 2) {
        _index = 0;
        return;
      }
      _calculatedChecksum += ch;
      break;

    default:
      if (_index == _frameLen + 2) {
        _checksum = ch << 8;
      }
      else if (_index == _frameLen + 2 + 1) {
        _checksum |= ch;

        if (_calculatedChecksum == _checksum) {
          _PMSstatus = STATUS_OK;

          //using payload16_t = std::array<uint16, payload.size() / 2>;

          // std::bit_cast requires GCC 11
          //payload16_t payload16 = (payload16_t&)payload;

          data = {
            // Standard Particles, CF=1.
            .PM_SP_UG_1_0 = makeWord(payload[0], payload[1]),
            .PM_SP_UG_2_5 = makeWord(payload[2], payload[3]),
            .PM_SP_UG_10_0 = makeWord(payload[4], payload[5]),
            // Atmospheric Environment.
            .PM_AE_UG_1_0 = makeWord(payload[6], payload[7]),
            .PM_AE_UG_2_5 = makeWord(payload[8], payload[9]),
            .PM_AE_UG_10_0 = makeWord(payload[10], payload[11])
          };

          /*
                    // Standard Particles, CF=1.
            .PM_SP_UG_1_0 = payload16[0],
            .PM_SP_UG_2_5 = payload16[1],
            .PM_SP_UG_10_0 = payload16[2],
            // Atmospheric Environment.
            .PM_AE_UG_1_0 = payload16[3],
            .PM_AE_UG_2_5 = payload16[4],
            .PM_AE_UG_10_0 = payload16[5]
          */
        }

        _index = 0;
        return;
      }
      else {
        _calculatedChecksum += ch;
        uint8_t payloadIndex = _index - 4;

        // Payload is common to all sensors (first 2x6 bytes).
        if _likely(payloadIndex < payload.size()) [[likely]] {
          payload[payloadIndex] = ch;
        }
      }

      break;
    }

    _index++;
  };

  inner_func();

  pms5003::payload = payload;
}


#pragma endregion

#pragma region SHT31 TMP_RH

namespace zephyr::temperature_humidity {
  namespace {
    namespace state {
      static i2c::address address = util::max_value<i2c::address>;
    }

    enum class command : uint16 {
      read_serial_number = 0x3780,

      read_status = 0xF32D,
      clear_status = 0x3041,

      heater_enable = 0x306D,
      heater_disable = 0x3066,

      soft_reset = 0x30A2,

      clock_stretch_h = 0x2C06,
      clock_stretch_m = 0x2C0D,
      clock_stretch_l = 0x2C10,

      polling_h = 0x2400,
      polling_m = 0x240B,
      polling_l = 0x2416,

      art = 0x2B32,

      fetch_data = 0xE000,
      stop_periodic = 0x3093,

      read_alr_limit_ls = 0xE102,
      read_alr_limit_lc = 0xE109,
      read_alr_limit_hs = 0xE11F,
      read_alr_limit_hc = 0xE114,

      write_alr_limit_hs = 0x611D,
      write_alr_limit_hc = 0x6116,
      write_alr_limit_lc = 0x610B,
      write_alr_limit_ls = 0x6100,

      no_sleep = 0x303E,
    };

    enum class repeatability {
      high,
      medium,
      low,
    };

    enum class mode {
      clock_stretch,
      polling,
    };

    enum class frequency {
      hz_half,
      hz_1,
      hz_2,
      hz_4,
      hz_10
    };

    template <typename T> requires (sizeof(T) == 2 && (std::is_same_v<T, command> || std::is_integral_v<T>))
    static error_code write_command(T command) {
      Wire.beginTransmission(state::address);
      Wire.write(uint32(command) >> 8);
      Wire.write(uint8(command));
      return error_code(-10 * Wire.endTransmission());
    }

    template <repeatability repeatability, frequency frequency>
    __attribute__((optimize("-Os"), const))
    static constexpr uint16 get_periodic_command() {
      uint16_t command = 0u;

      switch (frequency) {
      case frequency::hz_half:
          command = 0x2000u;
          switch (repeatability) {
          case repeatability::high:
              return command |= 0x32u;
          case repeatability::medium:
              return command |= 0x24u;
          case repeatability::low:
              return command |= 0x2Fu;
          default:
              __builtin_unreachable();
              break;
          }
          break;
      case frequency::hz_1:
          command = 0x2100u;
          switch (repeatability) {
          case repeatability::high:
              return command |= 0x30u;
          case repeatability::medium:
              return command |= 0x26u;
          case repeatability::low:
              return command |= 0x2Du;
          default:
              __builtin_unreachable();
              break;
          }
          break;
      case frequency::hz_2:
          command = 0x2200u;
          switch (repeatability) {
          case repeatability::high:
              return command |= 0x36u;
          case repeatability::medium:
              return command |= 0x20u;
          case repeatability::low:
              return command |= 0x2Bu;
          default:
              __builtin_unreachable();
              break;
          }
          break;
      case frequency::hz_4:
          command = 0x2300u;
          switch (repeatability) {
          case repeatability::high:
              return command |= 0x34u;
          case repeatability::medium:
              return command |= 0x22u;
          case repeatability::low:
              return command |= 0x29u;
          default:
              __builtin_unreachable();
              break;
          }
          break;
      case frequency::hz_10:
          command = 0x2700u;
          switch (repeatability) {
          case repeatability::high:
              return command |= 0x37u;
          case repeatability::medium:
              return command |= 0x21u;
          case repeatability::low:
              return command |= 0x2Au;
          default:
              __builtin_unreachable();
              break;
          }
          break;
      default:
          __builtin_unreachable();
          break;
      }
    }

    template <repeatability repeatability, frequency frequency>
    static error_code periodic_start() {
      const uint16 command = get_periodic_command<repeatability, frequency>();
      const auto error = write_command(command);
      return error;
    }

    static error_code periodic_stop() {
      return write_command(command::stop_periodic);
    }

    static error_code reset() {
      return write_command(command::soft_reset);
    }

    static error_code clear() {
      return write_command(command::clear_status);
    }

    struct alignas(uint8) [[gnu::packed]] read_struct final {
      uint16 data = {};
      uint8 checksum = uint8(-1);

      operator uint8 * () { return (uint8 *)this; }
      operator const uint8 * () const { return (uint8 *)this; }
    };
    static_assert(sizeof(read_struct) == 3);

    static error_code read_internal(array<uint16, 2> &data) {
      const auto byte_count = data.size() * 3;
      Wire.requestFrom(state::address, byte_count);

      bool error = false;

      for (uint16 &out : data) {
        read_struct temp = {};

        Wire.readBytes(temp, sizeof(temp));

        // Don't put the conditional in here to make sure that the wire-stream actually gets flushed.
        error = _unlikely(_unlikely(error) || _unlikely(crc::i2c::get(temp) != temp.checksum));

        out = __builtin_bswap16(temp.data);
      }

      return error ? error_code::crc_error : error_code::no_error;
    }

    static uint32 get_serial() {
      if _likely(write_command(command::read_serial_number) == error_code::no_error) [[likely]] {
        array<uint16, 2> buffer;
        if (read_internal(buffer) == error_code::no_error) {
          return (buffer[0] << 16) | buffer[1];
        }
      }
      else {
        debug::println("TMP_RH Failed to Initialize.");
      }

      return 0u;
    }

    static constexpr float calculate_temperature(uint16 rawValue) //
    {
      return ((175 * rawValue) - (65'535 * 45)) / 65'535.0f;
    }

    static constexpr float calculate_humidity(uint16 rawValue) //
    {
      return (100 * rawValue) / 65'535.0f;
    }
  }

  __attribute__((optimize("-Os")))
  error_code initialize(i2c::address address) {
    debug::printfln("Initializing %s...", name);

    state::address = address;

    const auto error = periodic_start<repeatability::high, frequency::hz_10>();
    if _unlikely(error != error_code::no_error) [[unlikely]] {
      debug::printfln("Failed to initialize TMP_RH: %d", error);
    }
    else {
      debug::printfln("Successfully Initialized %s.", name);
    }
    return error;
  }

  data read() {
    auto error = write_command(command::fetch_data);

    if _unlikely(error != error_code::no_error) [[unlikely]] {
     return data::make_error(error);
    }

    array<uint16, 2> buffer;
    error = read_internal(buffer);

    if _likely(error == error_code::no_error) [[likely]] {
      const float temperature = calculate_temperature(buffer[0]);
      const float humidity = std::roundf(calculate_humidity(buffer[1]));

      // validate values
      if _unlikely(
        !std::isfinite(temperature) ||
        !std::isfinite(humidity) ||
        humidity < std::numeric_limits<int>::min() ||
        humidity > std::numeric_limits<int>::max()
      ) {
        return data::make_error(error_code::unknown);
      }

      return {
        .temperature = temperature,
        .relative_humidity = int(humidity),
        .error = error_code::no_error
      };
    }
    else {
      return data::make_error(error);
    }
  }

#pragma endregion

#if 0
#pragma region PM25

  namespace particulate {
    namespace {
      static util::lazy<SoftwareSerial> serial;

      enum class mode {
        active,
        passive
      };

      enum class status {
        ok,
        waiting
      };

      struct default_times final : static_class {
        struct response final : static_class {
          enum values : uint16 {
            single = 1'000,
            total = 1'000 * 10,
            steady = 1'000 * 30
          };
        };
      };

      static mode current_mode = mode::active;
      static int last_value = -1;

      class raw_data final {
        struct values final {
          uint16_t particles_1_0;
          uint16_t particles_2_5;
          uint16_t particles_10_0;
        };
        public:
        // Standard Particles, CF=1
        values standard;
        // Atmospheric environment
        values atmospheric;
      };

      using command_array = std::array<uint8, 7>;

      static constexpr __attribute__((pure))
      command_array make_command(uint8 a0, uint8 a1, uint8 a2, uint8 a3, uint8 a4) {
        return { 0x42, 0x4D, a0, a1, a2, a3, a4 };
      }

      static status blocking_read(raw_data & __restrict data, uint16 timeout = default_times::response::single) {
        time_ms start = millis();

        status current_status = status::waiting;

        do {
          current_status = loopyloopy(data);
        } while (current_status != status::ok && millis() - start < timeout);

        return current_status;
      }

      static bool request_passive_read() {
        if _unlikely(current_mode != mode::passive) [[unlikely]] {
          return false;
        }

        static constexpr const auto command = make_command(0xE2, 0x00, 0x00, 0x01, 0x71);
        serial->write(command.data(), command.size());
      }

      static int read_raw() {
        if _unlikely(!request_passive_read()) [[unlikely]] {
          return last_value;
        }

        raw_data data;
        int value;
        const auto status = blocking_read(data);
        if _likely(status == status::ok) [[likely]] {
          last_value = value = data.atmospheric.particles_2_5;
        }
        else {
          debug::printfln("PM25 Fetch failed: %d", status);
          value = last_value;
        }

        return value;
      }
    }

    __attribute__((optimize("-Os")))
    bool initialize(int8 rx_pin, int8 tx_pin, int baud) {
      debug::println("Initializing PM25...");

      auto &stream = serial.initialize(rx_pin, tx_pin);
      stream.begin(baud);

      if _unlikely(read_raw() < 0) [[unlikely]] {
        debug::println("PM25 Sensor Failed to Initialize");
      }
      else {
        debug::println("PM25 Sensor Initialized.");
        // warm up 10s
      }
    }
  }

#pragma endregion
#endif
}

#pragma region CO2

void AirGradient::CO2_Init() { CO2_Init(D4, D3); }
void AirGradient::CO2_Init(int rx_pin, int tx_pin) {
  CO2_Init(rx_pin, tx_pin, 9'600);
}
void AirGradient::CO2_Init(int rx_pin, int tx_pin, int baudRate) {
  debugln("Initializing CO2...");
  auto &serial = _SoftSerial_CO2.initialize(rx_pin, tx_pin);
  serial.begin(baudRate);
}

bool AirGradient::CO2_InitJoin(bool delay) {
  if _unlikely(getCO2_Raw() < 0) [[unlikely]] {
    debugln("CO2 Sensor Failed to Initialize ");
    return false;
  }
  else {
    if (delay) {
      Serial.println("CO2 Successfully Initialized. Heating up for 10s.");
      yield_delay(10'000);
    }
    else {
      Serial.println("CO2 Successfully Initialized.");
    }
    return true;
  }
}

static int last_co2_result = -1;

int AirGradient::getCO2(int retryLimit) {
  int ctr = 0;
  int result_CO2;
  do {
    result_CO2 = getCO2_Raw();
  } while (result_CO2 < 0 && ctr++ < retryLimit);

  if _unlikely(result_CO2 < 0) [[unlikely]] {
    return last_co2_result;
  }
  return last_co2_result = result_CO2;
}

static bool WaitForAvailable(SoftwareSerial& serial, int count, uint16 timeout) {
  time_ms start = millis();
  int available;

  do {
    available = serial.available();
  } while (available < count && millis() - start < timeout);

  return available >= count;
}

int AirGradient::getCO2_Raw() {
  static constexpr const byte CO2Command[] = {0xFE, 0x44, 0x00, 0x08, 0x02, 0x9F, 0x25};

  auto &serial = *_SoftSerial_CO2;
  serial.write(CO2Command, sizeof(CO2Command));

  // Wait for function code since we might just get an exception.
  if _unlikely(!WaitForAvailable(serial, 2, 200)) [[unlikely]] {
    return -2; // for debugging
  }

  if _unlikely(serial.peek() == 0x86) [[unlikely]] {
    // exception
    serial.read();
    int8 exception_code = int8(serial.read());
    return -3;
  }

  if _unlikely(!WaitForAvailable(serial, 3, 200)) [[unlikely]] {
    return -2; // for debugging
  }

  struct alignas(uint8) [[gnu::packed]] data_header final {
    uint8 address;
    uint8 function;
    uint8 bytes;
  } header;

  static_assert(sizeof(data_header) == 3);

  std::array<uint8, 5> byte_data;

  serial.readBytes((uint8*)&header, sizeof(header));

  byte_data[0] = header.address;
  byte_data[1] = header.function;
  byte_data[2] = header.bytes;

  if _unlikely(!WaitForAvailable(serial, header.bytes + 4, 200)) [[unlikely]] {
    return -2; // for debugging
  }

  uint16 result = 0;
  switch (header.bytes) {
    case 0: [[unlikely]];
      break; //??
    case 1: [[unlikely]]; {
      result = serial.read();
      byte_data[3] = result;
    } break;
    case 2: [[likely]]; {
      uint8 left_byte = byte_data[3] = serial.read();
      uint8 right_byte = byte_data[4] = serial.read();
      result = uint16(left_byte << 8) | right_byte;
    } break;
    default: [[unlikely]]; {
      // Flush and return, I guess.
      if _unlikely(!WaitForAvailable(serial, 7, 200)) [[unlikely]] {
        return -2; // for debugging
      }

      for (int i = 0; i < header.bytes + 2; ++i) {
        serial.read();
      }
      return -3;
    }
  }

  uint16 checksum = 0;
  serial.readBytes((uint8 *)&checksum, sizeof(checksum));

  if (header.address != 0xFE) {
    return -1;
  }

  auto calculated_checksum = zephyr::crc::modbus::get((const uint8 *)&byte_data, sizeof(byte_data) - (2 - header.bytes));

  if _unlikely(calculated_checksum != checksum) [[unlikely]] {
    zephyr::debug::printfln("checksum mismatch: %04X != %04X", calculated_checksum, checksum);
    zephyr::debug::printfln("packet: [ %02X %02X %02X %02X %02X ]", byte_data[0], byte_data[1], byte_data[2], byte_data[3], byte_data[4]);
    zephyr::debug::printfln("header: address %04X function %04X size %04X", header.address, header.function, header.bytes);
    zephyr::debug::printfln("result: %04X", result);
    return -4; //crc mismatch
  }

  return result;
}

#pragma endregion
