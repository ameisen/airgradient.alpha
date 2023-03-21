#pragma once

#include <cstdint>
#include <array>
#include <limits>
#include <type_traits>
#include <utility>
#include <cassert>

#define _expect(expression, value_) \
  (__builtin_expect((expression), (value_)))
#define _likely(expression) \
  _expect(!!(expression), true)
#define _unlikely(expression) \
  _expect(!!(expression), false)

#define _pure __attribute__((pure))
#define _const __attribute__((const))
#define _leaf __attribute__((leaf))

namespace types {
  using uint8 = std::uint8_t;
  using uint16 = std::uint16_t;
  using uint32 = std::uint32_t;
  using uint64 = std::uint64_t;

  using int8 = std::int8_t;
  using int16 = std::int16_t;
  using int32 = std::int32_t;
  using int64 = std::int64_t;

  using uint = uint32;
  using time_ms = int64;
  using usize = uint32; // always truncates, but I don't think we can reasonably handle sizes larger than 2^32.
}

namespace util {
  template <typename T>
  static constexpr const T lowest_value = std::numeric_limits<T>::lowest();

  template <typename T>
  static constexpr const T min_value = std::numeric_limits<T>::min();

  template <typename T>
  static constexpr const T max_value = std::numeric_limits<T>::max();
}

namespace {
  using namespace types;

  using std::array;

  static constexpr inline uint8 operator ""_u8(unsigned long long int value_) {
    return uint8(value_);
  }
}
