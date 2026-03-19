#pragma once

/**
 * @file timing.h
 * @brief Zephyr timing / delay implementation
 *
 * Provides blocking delay functions with microsecond and millisecond
 * granularity, backed by Zephyr's kernel timing APIs. Inherits from
 * TimingBase (CRTP) for a platform-agnostic interface.
 *
 * Usage:
 * @code
 * #include <myl_utils/zephyr/timing.h>
 *
 * ZephyrTiming::DelayUs(100);     // busy-wait 100 µs
 * ZephyrTiming::DelayMs(50);      // sleep 50 ms (thread yields to scheduler)
 * @endcode
 *
 * @note DelayUs() uses k_busy_wait() — the CPU spins for the requested
 *       duration.  Prefer DelayMs() (which calls k_sleep) for longer
 *       waits so other threads can run.
 */

#include <myl_utils/timing.h>
#include <zephyr/kernel.h>

/**
 * @brief Zephyr timing implementation
 *
 * Uses k_busy_wait() for microsecond delays and k_sleep() for millisecond
 * delays (yields to the Zephyr scheduler).
 */
class ZephyrTiming : public TimingBase<ZephyrTiming> {
 public:
  /**
   * @brief Busy-wait for a number of microseconds
   *
   * Uses k_busy_wait() — the CPU spins.  Suitable for short,
   * timing-critical delays (sensor setup times, bus turnaround, etc.).
   *
   * @param us Delay in microseconds
   */
  static void DelayUs(uint32_t us) {
    k_busy_wait(us);
  }

  /**
   * @brief Sleep for a number of milliseconds (thread-friendly)
   *
   * Puts the calling thread to sleep via k_sleep(), allowing other threads
   * to run.  Not usable from ISR context.
   *
   * @param ms Delay in milliseconds
   */
  static void DelayMs(uint32_t ms) {
    k_sleep(K_MSEC(ms));
  }
};
