#pragma once

/**
 * @file pwm.h
 * @brief Abstract PWM output interface
 *
 * Provides a minimal, platform-agnostic interface for PWM outputs.
 * Platform implementations (Zephyr, STM32 HAL, etc.) inherit from
 * PwmOutput and provide the actual hardware access.
 *
 * Usage:
 * @code
 * auto &motor = get_motor_pwm();  // returns PwmOutput&
 *
 * motor.SetDutyPercent(50);       // 50% duty cycle
 * motor.SetDutyCycle(2500, 5000); // pulse=2500, period=5000 (in implementation units)
 * motor.SetPeriodAndDuty(20000, 1500);  // period=20ms, pulse=1.5ms (servo)
 * motor.Stop();
 * @endcode
 */

#include <cstdint>

/**
 * @brief Abstract base class for a PWM output channel
 *
 * Implementations must provide SetDutyCycle(), SetPeriodAndDuty(), and Stop().
 * Units for period and pulse are implementation-defined (typically nanoseconds
 * for Zephyr, timer ticks or microseconds for STM32).
 */
class PwmOutput {
 public:
  virtual ~PwmOutput() = default;

  /**
   * @brief Set PWM duty cycle with an existing period
   * @param pulse Pulse width (on-time) in implementation-defined units
   * @param period Full cycle period in implementation-defined units
   * @return true on success
   */
  virtual bool SetDutyCycle(uint32_t pulse, uint32_t period) = 0;

  /**
   * @brief Set PWM period and pulse width together
   * @param period_us Period in microseconds
   * @param pulse_us Pulse width (on-time) in microseconds
   * @return true on success
   */
  virtual bool SetPeriodAndDuty(uint32_t period_us, uint32_t pulse_us) = 0;

  /**
   * @brief Set duty cycle as a percentage (0–100)
   * @param percent Duty cycle percentage
   * @param period_us Period in microseconds (default: keep current period)
   * @return true on success
   */
  virtual bool SetDutyPercent(uint8_t percent, uint32_t period_us = 0) = 0;

  /**
   * @brief Stop PWM output (set pulse to 0)
   */
  virtual void Stop() = 0;
};
