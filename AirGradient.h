/*
  Test.h - Test library for Wiring - description
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// ensure this library description is only included once
#ifndef AirGradient_h
#define AirGradient_h

#include "Stream.h"
#include <Print.h>
#include <SoftwareSerial.h>
#include <optional>
#include <cstdint>
#include <limits>

#include "common.hpp"
#include "lazy.hpp"
#include "optional.hpp"

#define USING_SHT31 1

namespace zephyr {
  namespace i2c {
    using address = int8;

    static constexpr inline bool is_valid(address address) {
      return address >= 0;
    }
  }

  namespace temperature_humidity {
    static constexpr const char name[] = "SHT31";

    enum class error_code : int {
      no_error = 0,

      crc_error = -101,
      timeout_error = -102,

      param_wrong_mode = -501,
      param_wrong_repeatability = -502,
      param_wrong_frequency = -503,
      param_wrong_alert = -504,

      // Wire I2C translated error codes
      wire_i2c_data_too_log = -10,
      wire_i2c_received_nack_on_address = -20,
      wire_i2c_received_nack_on_data = -30,
      wire_i2c_unknow_error = -40,

      // Custom
      unknown = -1
    };

    struct data final {
      float temperature;
      int relative_humidity;
      error_code error;

      static inline data make_error(error_code error) {
        return {
          .temperature = std::nanf(""),
          .relative_humidity = -1,
          .error = error
        };
      }
    };

    extern error_code initialize(i2c::address address);
    extern data read();
  }
}

// ENUMS STRUCTS FOR CO2 START
struct CO2_READ_RESULT {
  int co2 = -1;
  bool success = false;
};
// ENUMS STRUCTS FOR CO2 END

// library interface description
class AirGradient {
  // user-accessible "public" interface
public:
  static constexpr const int baud_rate = 921'600;

  AirGradient(bool displayMsg = false, int baudRate = baud_rate);
  // void begin(int baudRate=9600);

  static void setOutput(Print &debugOut, bool verbose = true);

  static constexpr const int PMS_WarmUp = 10'000;

  void PMS_Init(void);
  void PMS_Init(int, int);
  void PMS_Init(int, int, int);
  bool PMS_InitJoin(bool);

  bool _debugMsg;

  // PMS VARIABLES PUBLIC_START
  static const uint16_t SINGLE_RESPONSE_TIME = 1000;
  static const uint16_t TOTAL_RESPONSE_TIME = 1000 * 10;
  static const uint16_t STEADY_RESPONSE_TIME = 1000 * 30;

  static const uint16_t BAUD_RATE = 9600;

  struct pm_raw_data final {
    // Standard Particles, CF=1
    uint16_t PM_SP_UG_1_0;
    uint16_t PM_SP_UG_2_5;
    uint16_t PM_SP_UG_10_0;

    // Atmospheric environment
    uint16_t PM_AE_UG_1_0;
    uint16_t PM_AE_UG_2_5;
    uint16_t PM_AE_UG_10_0;
  };

  void sleep();
  void wakeUp();
  void activeMode();
  void passiveMode();

  void requestRead();
  bool read_PMS(pm_raw_data & __restrict data);
  bool readUntil(pm_raw_data & __restrict data, uint16_t timeout = SINGLE_RESPONSE_TIME);
  int getPM2();
  int getPM2_Raw();

  // PMS VARIABLES PUBLIC_END

  // CO2 VARIABLES PUBLIC START
  static constexpr const int CO2_WarmUp = 10'000;

  void CO2_Init();
  void CO2_Init(int, int);
  void CO2_Init(int, int, int);
  bool CO2_InitJoin(bool);
  int getCO2(int retryLimit = 5);
  int getCO2_Raw();
  util::lazy<SoftwareSerial> _SoftSerial_CO2;

  // CO2 VARIABLES PUBLIC END

  // library-accessible "private" interface
private:
  // PMS VARIABLES PRIVATE START
  enum STATUS { STATUS_WAITING, STATUS_OK };
  enum MODE { MODE_ACTIVE, MODE_PASSIVE };

  STATUS _PMSstatus;
  MODE _mode = MODE_ACTIVE;

  uint8_t _index = 0;
  uint16_t _frameLen;
  uint16_t _checksum;
  uint16_t _calculatedChecksum;
  util::lazy<SoftwareSerial> _SoftSerial_PMS;
  void loop(pm_raw_data & __restrict data);
  // PMS VARIABLES PRIVATE END

public:
  template <typename T>
  inline void debugln(const T &value) const {
    if _likely(!_debugMsg) [[likely]] {
      //return;
    }

    Serial.println(value);
  }

  template <typename TF, typename T>
  inline void debugfln(const TF &format, const T &value) const {
    if _likely(!_debugMsg) [[likely]] {
      //return;
    }

    Serial.printf(format, value);
    Serial.println();
  }
};

#endif
