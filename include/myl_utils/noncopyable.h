#pragma once

template <typename T>
class NonCopyable {
 protected:
  /**
   * Disallow construction of NonCopyable objects from outside of its hierarchy.
   */
  NonCopyable() = default;
  /**
   * Disallow destruction of NonCopyable objects from outside of its hierarchy.
   */
  ~NonCopyable() = default;

 public:
  /**
   * Define copy constructor as deleted. Any attempt to copy construct
   * a NonCopyable will fail at compile time.
   */
  NonCopyable(const NonCopyable &) = delete;

  /**
   * Define copy assignment operator as deleted. Any attempt to copy assign
   * a NonCopyable will fail at compile time.
   */
  NonCopyable &operator=(const NonCopyable &) = delete;
};