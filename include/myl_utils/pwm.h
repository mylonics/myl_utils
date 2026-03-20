#pragma once

/**
 * @file pwm.h
 * @brief Abstract PWM output interface (CRTP — zero virtual overhead)
 *
 * Provides a minimal, platform-agnostic interface for PWM outputs using
 * the Curiously Recurring Template Pattern (CRTP). Platform implementations
 * (Zephyr, STM32 HAL, etc.) inherit from PwmOutputBase and provide the
 * actual hardware access — all calls are inlined at compile time.
 *
 * Usage:
 * @code
 * auto &motor = get_motor_pwm();  // returns concrete type
 *
 * motor.SetDutyPercent(50);       // 50% duty cycle
 * motor.SetDutyCycle(2500, 5000); // pulse=2500, period=5000 (in implementation units)
 * motor.SetPeriodAndDuty(20000, 1500);  // period=20ms, pulse=1.5ms (servo)
 * motor.Stop();
 *
 * // Generic code uses templates (zero-cost):
 * template <typename Pwm>
 * void set_speed(Pwm &pwm, uint8_t pct) { pwm.SetDutyPercent(pct); }
 * @endcode
 */

#include <cstdint>

namespace myl_utils {

/**
 * @brief CRTP base class for a PWM output channel
 *
 * Implementations must provide SetDutyCycle(), SetPeriodAndDuty(),
 * SetDutyPercent(), and Stop() as public non-virtual methods.
 * Units for period and pulse are implementation-defined (typically nanoseconds
 * for Zephyr, timer ticks or microseconds for STM32).
 *
 * @tparam Derived The concrete implementation class (CRTP parameter)
 */
template <typename Derived>
class PwmOutputBase {
 public:
  /**
   * @brief Set PWM duty cycle with an existing period
   * @param pulse Pulse width (on-time) in implementation-defined units
   * @param period Full cycle period in implementation-defined units
   * @return true on success
   */
  bool SetDutyCycle(uint32_t pulse, uint32_t period) {
    return derived().SetDutyCycle(pulse, period);
  }

  /**
   * @brief Set PWM period and pulse width together
   * @param period_us Period in microseconds
   * @param pulse_us Pulse width (on-time) in microseconds
   * @return true on success
   */
  bool SetPeriodAndDuty(uint32_t period_us, uint32_t pulse_us) {
    return derived().SetPeriodAndDuty(period_us, pulse_us);
  }

  /**
   * @brief Set duty cycle as a percentage (0–100)
   * @param percent Duty cycle percentage
   * @param period_us Period in microseconds (default: keep current period)
   * @return true on success
   */
  bool SetDutyPercent(uint8_t percent, uint32_t period_us = 0) {
    return derived().SetDutyPercent(percent, period_us);
  }

  /**
   * @brief Stop PWM output (set pulse to 0)
   */
  void Stop() { derived().Stop(); }

 protected:
  PwmOutputBase() = default;
  ~PwmOutputBase() = default;

 private:
  Derived &derived() { return static_cast<Derived &>(*this); }
  const Derived &derived() const { return static_cast<const Derived &>(*this); }
};

}  // namespace myl_utils
