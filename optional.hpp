#pragma once

#include "common.hpp"

namespace util {
	template <typename T>
	class alignas(T) optional final {
		T value_ = {};
		bool has_value_ = false;

  public:
		optional() = default;
		optional(T _value) :
			value_(_value),
      has_value_(true)
		{
		}

    template <typename U = T> requires std::is_copy_constructible_v<U>
    optional & operator = (const T &other) {
      value_ = other;
      has_value_ = true;
      return *this;
    }
    template <typename U = T> requires std::is_move_constructible_v<U>
    optional & operator = (T &&other) {
      value_ = std::move(other);
      has_value_ = true;
      return *this;
    }

    template <typename U = T> requires std::is_copy_constructible_v<U>
    optional & operator = (const optional<T> &other) {
      value_ = other.value_;
      has_value_ = other.has_value_;
      return *this;
    }
    template <typename U = T> requires std::is_move_constructible_v<U>
    optional & operator = (optional<T> &&other) {
      value_ = std::move(other.value_);
      has_value_ = other.has_value_;
      return *this;
    }

    T value() const __restrict {
      assert(has_value_);
      return value_;
    }

    bool has_value() const __restrict {
      return has_value_;
    }

		operator bool () const __restrict {
			return has_value_;
		}

		explicit operator T & () __restrict {
			assert(has_value_);
			return value_;
		}

		explicit operator T () const __restrict {
			assert(has_value_);
			return value_;
		}

		T & operator * () __restrict {
			assert(has_value_);
			return value_;
		}

		T operator * () const __restrict {
			assert(has_value_);
			return value_;
		}
	};
}
