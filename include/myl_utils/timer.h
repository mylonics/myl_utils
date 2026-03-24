#pragma once

/**
 * @file timer.h
 * @brief Abstract hardware timer interface (CRTP — zero virtual overhead)
 *
 * Provides a minimal, platform-agnostic interface for one-shot hardware
 * timers using the Curiously Recurring Template Pattern (CRTP). Platform
 * implementations (STM32 HAL, Zephyr, etc.) inherit from HwTimerBase and
 * provide the actual hardware access — all calls are inlined at compile time.
 *
 * Usage:
 * @code
 * auto &timer = get_platform_timer();  // returns concrete type
 *
 * timer.SetCallback([] { do_something(); });
 * timer.StartUs(500);          // one-shot fires after 500 µs
 * timer.StartWithPeriod(1000); // one-shot with raw tick count
 * timer.Stop();                // cancel a running timer
 *
 * // In the timer's ISR:
 * timer.OnInterrupt();
 *
 * // Generic code uses templates (zero-cost):
 * template <typename Timer>
 * void delayed_trigger(Timer &t, uint32_t us) { t.StartUs(us); }
 * @endcode
 */

#include "config.h"
#include <cstdint>

namespace myl_utils {

/**
 * @brief CRTP base class for one-shot hardware timer operations
 *
 * Implementations must provide:
 *  - bool StartUs(uint32_t time_us)
 *  - bool StartWithPeriod(uint32_t period)
 *  - void Stop()
 *  - void OnInterrupt()
 *  - void SetCallback(Callback cb)
 *
 * @tparam Derived The concrete implementation class (CRTP parameter)
 */
template <typename Derived>
class HwTimerBase {
 public:
  /// Callback type — invoked from interrupt context
  using Callback = void (*)();

  /**
   * @brief Start a one-shot timer that fires after @p time_us microseconds
   * @param time_us Delay in microseconds (must be > 0)
   * @return true if the timer was started
   */
  MYL_NOINLINE bool StartUs(uint32_t time_us) {
    return derived().StartUs(time_us);
  }

  /**
   * @brief Start a one-shot timer with a raw period value
   *
   * The meaning of @p period is implementation-defined (typically timer ticks).
   *
   * @param period Raw tick count
   * @return true if the timer was started
   */
  MYL_NOINLINE bool StartWithPeriod(uint32_t period) {
    return derived().StartWithPeriod(period);
  }

  /**
   * @brief Cancel a running one-shot timer
   *
   * Disables the timer and its interrupt. Safe to call even if the
   * timer is not currently running.
   */
  MYL_NOINLINE void Stop() { derived().Stop(); }

  /**
   * @brief Call from the timer's interrupt handler
   *
   * Disables the timer (making it one-shot) and invokes the user callback.
   */
  MYL_NOINLINE void OnInterrupt() { derived().OnInterrupt(); }

  /**
   * @brief Set or replace the completion callback
   * @param cb Function called when the one-shot fires (ISR context)
   */
  MYL_NOINLINE void SetCallback(Callback cb) { derived().SetCallback(cb); }

 protected:
  HwTimerBase() = default;
  ~HwTimerBase() = default;

 private:
  Derived &derived() { return static_cast<Derived &>(*this); }
  const Derived &derived() const { return static_cast<const Derived &>(*this); }
};

}  // namespace myl_utils
