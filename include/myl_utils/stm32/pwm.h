#pragma once

/**
 * @file pwm.h
 * @brief STM32 HAL PWM implementation
 *
 * Provides a PwmOutput implementation backed by STM32 HAL TIM PWM functions.
 * The timer and channel must already be configured by CubeMX / HAL_TIM_PWM_Init().
 *
 * @note Requires the STM32 HAL header to be included before this file.
 *       Typically satisfied by including "main.h" or the HAL umbrella header.
 *
 * Usage:
 * @code
 * #include "main.h"
 * #include <myl_utils/stm32/pwm.h>
 *
 * extern TIM_HandleTypeDef htim3;
 * Stm32PwmOutput motor(&htim3, TIM_CHANNEL_1);
 *
 * motor.Start();                            // begin PWM output
 * motor.SetDutyPercent(50);                 // 50% duty at CubeMX-configured period
 * motor.SetPeriodAndDuty(20000, 1500);      // 20ms period, 1.5ms pulse (servo)
 * motor.Stop();
 * @endcode
 */

#include <myl_utils/pwm.h>

#ifndef HAL_TIM_MODULE_ENABLED
#error "Include your STM32 HAL header (e.g. stm32f4xx_hal.h) before myl_utils/stm32/pwm.h"
#endif

namespace myl_utils {

/**
 * @brief STM32 HAL PWM output channel
 *
 * Wraps a TIM_HandleTypeDef and channel. The timer's ARR (auto-reload) value
 * determines the period; the CCR (capture/compare) value determines the pulse.
 *
 * The timer prescaler and clock source must be configured by CubeMX.
 * This class manipulates ARR and CCR registers directly for period/duty changes.
 */
class Stm32PwmOutput : public PwmOutputBase<Stm32PwmOutput> {
 public:
  Stm32PwmOutput(const Stm32PwmOutput &) = delete;
  Stm32PwmOutput &operator=(const Stm32PwmOutput &) = delete;
  Stm32PwmOutput(Stm32PwmOutput &&) = delete;
  Stm32PwmOutput &operator=(Stm32PwmOutput &&) = delete;

 protected:
  TIM_HandleTypeDef *htim_;
  uint32_t channel_;
  uint32_t timer_clock_mhz_;  ///< Timer input clock in MHz (after prescaler)
  volatile uint32_t *ccr_;    ///< Cached pointer to the CCR register (resolved once at construction)
  bool running_{};

  /// Resolve CCR register pointer from channel (called once at construction)
  static volatile uint32_t *ResolveCcr(TIM_HandleTypeDef *htim, uint32_t channel) {
    switch (channel) {
      case TIM_CHANNEL_1: return &htim->Instance->CCR1;
      case TIM_CHANNEL_2: return &htim->Instance->CCR2;
      case TIM_CHANNEL_3: return &htim->Instance->CCR3;
      case TIM_CHANNEL_4: return &htim->Instance->CCR4;
      default: return &htim->Instance->CCR1;
    }
  }

 public:
  /**
   * @brief Construct an STM32 PWM output
   * @param htim Pointer to HAL timer handle (e.g. &htim3)
   * @param channel Timer channel (e.g. TIM_CHANNEL_1)
   * @param timer_clock_mhz Timer clock in MHz after prescaler
   *        (e.g., if APB clock = 84MHz and prescaler = 83, then timer_clock_mhz = 1)
   *        Used to convert microseconds to timer ticks. Default: 1 MHz (1 tick = 1 µs).
   */
  Stm32PwmOutput(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t timer_clock_mhz = 1)
      : htim_(htim), channel_(channel), timer_clock_mhz_(timer_clock_mhz),
        ccr_(ResolveCcr(htim, channel)) {}

  /**
   * @brief Start PWM output
   * @return true on success
   */
  bool Start() {
    running_ = (HAL_TIM_PWM_Start(htim_, channel_) == HAL_OK);
    return running_;
  }

  bool SetDutyCycle(uint32_t pulse, uint32_t period) {
    htim_->Instance->ARR = period - 1;
    *ccr_ = pulse;
    return true;
  }

  bool SetPeriodAndDuty(uint32_t period_us, uint32_t pulse_us) {
    uint32_t period_ticks = period_us * timer_clock_mhz_;
    uint32_t pulse_ticks = pulse_us * timer_clock_mhz_;
    htim_->Instance->ARR = period_ticks - 1;
    *ccr_ = pulse_ticks;
    return true;
  }

  bool SetDutyPercent(uint8_t percent, uint32_t period_us = 0) {
    if (percent > 100) percent = 100;
    if (period_us > 0) {
      uint32_t period_ticks = period_us * timer_clock_mhz_;
      htim_->Instance->ARR = period_ticks - 1;
    }
    uint32_t arr = htim_->Instance->ARR + 1;
    *ccr_ = (arr * percent) / 100U;
    return true;
  }

  void Stop() {
    HAL_TIM_PWM_Stop(htim_, channel_);
    running_ = false;
  }

  /// Check if PWM output is currently running
  bool IsRunning() const { return running_; }

  /// Direct access to set the CCR (compare) value in raw timer ticks
  void SetCompareTicks(uint32_t ticks) { *ccr_ = ticks; }

  /// Direct access to set ARR (auto-reload) value in raw timer ticks
  void SetPeriodTicks(uint32_t ticks) { htim_->Instance->ARR = ticks - 1; }
};

}  // namespace myl_utils
