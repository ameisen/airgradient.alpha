#pragma once

#include "common.hpp"
#include <cassert>

namespace util {
	template <typename T>
	class alignas(T) [[gnu::packed]] lazy final {
		std::aligned_storage_t<sizeof(T), std::alignment_of_v<T>> storage_ = {};
		bool initialized_ = false;

		void destroy() {
			if _likely(initialized_) [[likely]] {
				std::destroy_at(&value());
				initialized_ = false;
			}
		}

public:
		lazy() = default;

    template <typename U = T> requires std::is_copy_constructible_v<U>
		lazy(const lazy<T> &value) {
			if _unlikely(!value.initialized_) [[unlikely]] {
				return;
			}
			new (&storage_) T(value.value());
		}

		template <typename U = T> requires std::is_move_constructible_v<U>
		lazy(lazy<T> &&value) {
			if _unlikely(!value.initialized_) [[unlikely]] {
				return;
			}
			new (&storage_) T(std::move(value.value()));
		}

		template <typename U = T> requires std::is_copy_constructible_v<U>
		lazy(const T &value) {
			new (&storage_) T(value.value());
		}

		template <typename U = T> requires std::is_move_constructible_v<U>
		lazy(T &&value) {
			new (&storage_) T(std::move(value.value()));
		}

		~lazy() {
			destroy();
		}

		template <typename U = T> requires std::is_copy_assignable_v<U>
		lazy<T> & operator = (const lazy<T> &value) {
			if _likely(value.initialized_) [[likely]] {
				value() = value.value();
			}
			else {
				destroy();
			}
			return *this;
		}

		template <typename U = T> requires std::is_move_assignable_v<U>
		lazy<T> & operator = (lazy<T> &&value) {
			if _likely(value.initialized_) [[likely]] {
				value() = std::move(value.value());
			}
			else {
				destroy();
			}
			return *this;
		}

		template <typename U = T> requires std::is_copy_assignable_v<U>
		lazy<T> & operator = (const T &value) {
			value() = value;
			return *this;
		}

		template <typename U = T> requires std::is_move_assignable_v<U>
		lazy<T> & operator = (T &&value) {
			value() = std::move(value);
			return *this;
		}

		operator bool() const _const {
			return _likely(initialized_);
		}

		bool initialized() const _const {
			return _likely(initialized_);
		}

		template <typename... Args>
		T& initialize(Args&& ...args) {
			assert(!initialized_);

			new (&storage_) T(std::forward<Args>(args)...);
			initialized_ = true;

      return value();
		}

		T& value() _const {
      assert(initialized_);
			return *std::launder(reinterpret_cast<T*>(&storage_));
		}

		const T& value() const _const {
      assert(initialized_);
			return *std::launder(reinterpret_cast<const T*>(&storage_));
		}

		T* operator -> () _const {
      assert(initialized_);
			return &value();
		}

		const T* operator -> () const _const {
      assert(initialized_);
			return &value();
		}

		T& operator * () _const {
      assert(initialized_);
			return value();
		}

		const T& operator * () const _const {
      assert(initialized_);
			return value();
		}
	};
}
