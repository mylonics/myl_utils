#pragma once

/**
 * @file timing.h
 * @brief Abstract timing / delay interface (CRTP — zero virtual overhead)
 *
 * Provides a minimal, platform-agnostic interface for blocking delays using
 * the Curiously Recurring Template Pattern (CRTP). Platform implementations
 * (Zephyr, STM32 HAL, etc.) inherit from TimingBase and provide the
 * actual delay mechanism — all calls are inlined at compile time.
 *
 * Usage:
 * @code
 * auto &timer = get_platform_timer();  // returns concrete type
 *
 * timer.DelayUs(100);    // busy-wait 100 µs
 * timer.DelayMs(50);     // delay 50 ms (may yield on RTOS platforms)
 *
 * // Generic code uses templates (zero-cost):
 * template <typename Timer>
 * void sensor_setup() { Timer::DelayUs(10); }
 * @endcode
 */

#include <cstdint>

namespace myl_utils {

/**
 * @brief CRTP base class for timing / delay operations
 *
 * Implementations must provide static DelayUs(uint32_t) and DelayMs(uint32_t)
 * methods.
 *
 * @tparam Derived The concrete implementation class (CRTP parameter)
 */
template <typename Derived>
class TimingBase {
 public:
  /**
   * @brief Busy-wait for a number of microseconds
   * @param us Delay in microseconds
   */
  static void DelayUs(uint32_t us) { Derived::DelayUs(us); }

  /**
   * @brief Delay for a number of milliseconds
   *
   * Behaviour is platform-defined: may busy-wait (bare-metal) or yield
   * to a scheduler (RTOS).
   *
   * @param ms Delay in milliseconds
   */
  static void DelayMs(uint32_t ms) { Derived::DelayMs(ms); }

 protected:
  TimingBase() = default;
  ~TimingBase() = default;
};

}  // namespace myl_utils
