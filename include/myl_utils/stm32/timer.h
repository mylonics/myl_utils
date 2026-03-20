#pragma once

/**
 * @file timer.h
 * @brief STM32 HAL hardware timer utility (one-shot / retriggerable)
 *
 * Provides a lightweight wrapper around an STM32 HAL TIM handle for
 * one-shot timer operations with interrupt-driven completion callbacks.
 * Automatically calculates prescaler and period for arbitrary microsecond
 * delays, supporting durations well beyond the 16-bit counter limit.
 *
 * @note Requires the STM32 HAL header to be included before this file.
 *       Typically satisfied by including "main.h" or the HAL umbrella header.
 *
 * Usage:
 * @code
 * #include "main.h"
 * #include <myl_utils/stm32/timer.h>
 *
 * extern TIM_HandleTypeDef htim6;
 *
 * // Timer with 60 MHz input clock
 * myl_utils::Stm32HwTimer timer(&htim6, 60, [] { do_something(); });
 *
 * timer.StartUs(500);     // fire callback after 500 µs
 * timer.StartUs(100000);  // fire callback after 100 ms  (prescaler auto-adjusts)
 *
 * // Call from TIM6 update IRQ handler:
 * void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *   if (htim == &htim6) timer.OnInterrupt();
 * }
 * @endcode
 */

#include <myl_utils/noncopyable.h>
#include <myl_utils/timer.h>
#include <cstdint>

#ifndef HAL_TIM_MODULE_ENABLED
#error "Include your STM32 HAL header (e.g. stm32f4xx_hal.h) before myl_utils/stm32/timer.h"
#endif

namespace myl_utils {

/**
 * @brief STM32 HAL one-shot hardware timer
 *
 * Wraps a TIM_HandleTypeDef for interrupt-driven one-shot delays.
 * The timer counts up from a calculated offset to overflow (65535),
 * generating an update interrupt exactly once.
 *
 * The prescaler is automatically recalculated when the requested delay
 * exceeds what the current prescaler can represent in 16 bits.
 *
 * @note The HAL timer must be pre-configured (CubeMX) with NVIC enabled
 *       for the update interrupt. This class manages enable/disable and
 *       interrupt flags — do **not** call HAL_TIM_Base_Start_IT() externally.
 */
class Stm32HwTimer : public HwTimerBase<Stm32HwTimer>,
                     public NonCopyable<Stm32HwTimer> {
 public:
  /// Callback type — invoked from interrupt context
  using Callback = void (*)();

  /**
   * @brief Construct a hardware timer wrapper
   * @param htim         Pointer to HAL timer handle (e.g. &htim6)
   * @param timer_clock_mhz Timer input clock in MHz **before** any prescaler
   *                     (e.g. 60 for a 60 MHz APB timer clock)
   * @param callback     Function called when the one-shot fires (ISR context)
   */
  Stm32HwTimer(TIM_HandleTypeDef *htim, uint32_t timer_clock_mhz,
               Callback callback = nullptr)
      : htim_(htim), timer_clock_mhz_(timer_clock_mhz), callback_(callback) {}

  // -- Configuration --------------------------------------------------------

  /// Set or replace the completion callback
  void SetCallback(Callback cb) { callback_ = cb; }

  // -- One-shot operations --------------------------------------------------

  /**
   * @brief Start a one-shot timer that fires after @p time_us microseconds
   *
   * Automatically recalculates the prescaler if the requested duration
   * cannot fit in the 16-bit period register with the current prescaler.
   *
   * @param time_us Delay in microseconds (must be > 0)
   * @return true if the timer was started
   */
  bool StartUs(uint32_t time_us) {
    uint32_t period = CalculateAndSetPrescaler(time_us);
    return StartWithPeriod(period);
  }

  /**
   * @brief Start a one-shot timer with a raw period value
   *
   * The timer counts from (65535 − period) up to overflow. The effective
   * delay depends on the current prescaler setting.
   *
   * @param period Raw tick count (1 – 65535)
   * @return true if the timer was started
   */
  bool StartWithPeriod(uint32_t period) {
    // Disable timer to prevent any pending triggers
    __HAL_TIM_DISABLE(htim_);

    // Clear any pending interrupt flags
    __HAL_TIM_CLEAR_IT(htim_, TIM_IT_UPDATE);

    // Set counter value (counting up to overflow)
    htim_->Instance->CNT = kCounterMax - period;

    // Enable interrupt then start the timer
    __HAL_TIM_ENABLE_IT(htim_, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(htim_);
    return true;
  }

  /**
   * @brief Cancel a running one-shot timer
   *
   * Disables the timer and its update interrupt. Safe to call even if the
   * timer is not currently running.
   */
  void Stop() {
    __HAL_TIM_DISABLE(htim_);
    __HAL_TIM_CLEAR_IT(htim_, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE_IT(htim_, TIM_IT_UPDATE);
  }

  /**
   * @brief Call from the timer's update IRQ handler
   *
   * Disables the timer (making it one-shot), clears the interrupt flag,
   * and invokes the user callback.
   */
  void OnInterrupt() {
    // Disable timer to make it one-shot
    __HAL_TIM_DISABLE(htim_);
    // Clear interrupt flag
    __HAL_TIM_CLEAR_IT(htim_, TIM_IT_UPDATE);
    // Disable interrupt
    __HAL_TIM_DISABLE_IT(htim_, TIM_IT_UPDATE);

    if (callback_) {
      callback_();
    }
  }

  // -- Prescaler calculation ------------------------------------------------

  /**
   * @brief Calculate the optimal prescaler and period for a given delay
   *
   * Algorithm:
   *  - With prescaler = (clock_mhz − 1) each tick is 1 µs.
   *  - If time_us fits in 16 bits, use that directly.
   *  - Otherwise, increase the prescaler so the period fits in 16 bits
   *    while preserving as much timing accuracy as possible.
   *
   * The prescaler is only written to hardware when it differs from the
   * current setting, avoiding an unnecessary HAL_TIM_Base_Init() call.
   *
   * @param time_us Desired delay in microseconds
   * @return Period value in timer ticks (suitable for StartWithPeriod())
   */
  uint32_t CalculateAndSetPrescaler(uint32_t time_us) {
    if (time_us == 0) time_us = 1;

    // Convert to raw timer ticks (uint64_t to avoid overflow above ~71 s @ 60 MHz).
    // Using the full clock resolution (prescaler as low as possible) gives
    // sub-microsecond tick precision for short delays.
    const uint64_t total_ticks =
        static_cast<uint64_t>(time_us) * timer_clock_mhz_;

    // Minimum prescaler that keeps the period within 16 bits:
    // prescaler = ceil(total_ticks / 65535) − 1
    const uint32_t prescaler = static_cast<uint32_t>(
        (total_ticks + kCounterMax - 1) / kCounterMax - 1);

    // period = total_ticks / (prescaler + 1)
    const uint32_t period = static_cast<uint32_t>(
        total_ticks / (prescaler + 1));

    // Only reconfigure hardware when prescaler actually changes
    if (prescaler != htim_->Init.Prescaler) {
      htim_->Init.Prescaler = prescaler;
      HAL_TIM_Base_Init(htim_);
    }

    return period;
  }

  // -- Accessors ------------------------------------------------------------

  /// Get the underlying HAL timer handle
  TIM_HandleTypeDef *Handle() const { return htim_; }

  /// Get the configured timer input clock in MHz
  uint32_t ClockMhz() const { return timer_clock_mhz_; }

 private:
  static constexpr uint32_t kCounterMax = 65535;  ///< 16-bit counter maximum

  TIM_HandleTypeDef *htim_;
  uint32_t timer_clock_mhz_;
  Callback callback_;
};

}  // namespace myl_utils
