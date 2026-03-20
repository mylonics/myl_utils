#pragma once

/**
 * @file timing.h
 * @brief STM32 timing / delay implementation
 *
 * Provides blocking delay functions with microsecond and millisecond
 * granularity using the STM32 HAL and the ARM DWT cycle counter.
 * Inherits from TimingBase (CRTP) for a platform-agnostic interface.
 *
 * @note Requires the STM32 HAL header to be included before this file
 *       (e.g. "stm32f4xx_hal.h" or CubeMX-generated "main.h").
 *
 * Usage:
 * @code
 * #include "main.h"  // STM32 HAL
 * #include <myl_utils/stm32/timing.h>
 *
 * Stm32Timing::DelayUs(100);     // busy-wait 100 µs  (DWT auto-inits on first call)
 * Stm32Timing::DelayMs(50);      // busy-wait 50 ms   (SysTick-based via HAL_Delay)
 * @endcode
 */

#include <myl_utils/timing.h>

#ifndef HAL_MODULE_ENABLED
#error "Include your STM32 HAL header (e.g. stm32f4xx_hal.h) before myl_utils/stm32/timing.h"
#endif

namespace myl_utils {

/**
 * @brief STM32 timing implementation using DWT + HAL_Delay
 *
 * The DWT cycle counter is automatically enabled on the first call to
 * DelayUs().  Millisecond delays use HAL_Delay() (SysTick-based polling).
 */
class Stm32Timing : public TimingBase<Stm32Timing> {
 public:
  /**
   * @brief Busy-wait for a number of microseconds
   *
   * Uses the ARM DWT cycle counter for sub-millisecond precision.
   * The DWT is automatically initialised on the first call.
   *
   * @param us Delay in microseconds
   */
  static void DelayUs(uint32_t us) {
    static const bool init_done = (Init(), true);
    (void)init_done;
    const uint32_t start = DWT->CYCCNT;
    const uint32_t ticks = us * (SystemCoreClock / 1'000'000);
    while ((DWT->CYCCNT - start) < ticks) {
      // spin
    }
  }

  /**
   * @brief Busy-wait for a number of milliseconds (SysTick-based)
   *
   * Thin wrapper around HAL_Delay() — polls uwTick, requires SysTick ISR.
   *
   * @param ms Delay in milliseconds
   */
  static void DelayMs(uint32_t ms) {
    HAL_Delay(ms);
  }

 private:
  /**
   * @brief Enable the DWT cycle counter
   *
   * Called automatically on first use of DelayUs().
   */
  static void Init() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
};

}  // namespace myl_utils
