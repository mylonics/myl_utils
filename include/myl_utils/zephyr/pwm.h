#pragma once

/**
 * @file pwm.h
 * @brief Zephyr PWM implementation
 *
 * Provides a PwmOutput implementation backed by Zephyr's pwm_dt_spec API.
 * All time units are nanoseconds internally (Zephyr convention), but the
 * PwmOutput interface methods accept microseconds for portability.
 *
 * Usage:
 * @code
 * // From devicetree:
 * //   pwmleds { compatible = "pwm-leds"; pwm_led0 { pwms = <&pwm0 0 PWM_MSEC(20) PWM_POLARITY_NORMAL>; }; };
 * static const struct pwm_dt_spec pwm_spec = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led0));
 * ZephyrPwmOutput led_pwm(pwm_spec);
 *
 * led_pwm.SetDutyPercent(75);                  // 75% duty at DT-defined period
 * led_pwm.SetPeriodAndDuty(20000, 1500);       // 20ms period, 1.5ms pulse (servo)
 * led_pwm.Stop();
 * @endcode
 */

#include <myl_utils/noncopyable.h>
#include <myl_utils/pwm.h>
#include <zephyr/drivers/pwm.h>

namespace myl_utils {

/**
 * @brief Zephyr PWM output channel
 *
 * Wraps a pwm_dt_spec. The devicetree-defined period is used as the
 * default when SetDutyPercent() is called without an explicit period.
 */
class ZephyrPwmOutput : public PwmOutputBase<ZephyrPwmOutput>,
                        NonCopyable<ZephyrPwmOutput> {
 protected:
  const struct pwm_dt_spec spec_;
  uint32_t current_period_ns_;  ///< Cached period in nanoseconds

 public:

  /**
   * @brief Construct a Zephyr PWM output
   * @param spec Devicetree PWM spec (from PWM_DT_SPEC_GET)
   */
  explicit ZephyrPwmOutput(const struct pwm_dt_spec &spec)
      : spec_(spec), current_period_ns_(spec.period) {}

  bool SetDutyCycle(uint32_t pulse, uint32_t period) {
    current_period_ns_ = period;
    return pwm_set_dt(&spec_, period, pulse) == 0;
  }

  bool SetPeriodAndDuty(uint32_t period_us, uint32_t pulse_us) {
    uint32_t period_ns = period_us * 1000U;
    uint32_t pulse_ns = pulse_us * 1000U;
    current_period_ns_ = period_ns;
    return pwm_set_dt(&spec_, period_ns, pulse_ns) == 0;
  }

  bool SetDutyPercent(uint8_t percent, uint32_t period_us = 0) {
    if (percent > 100) percent = 100;
    if (period_us > 0) {
      current_period_ns_ = period_us * 1000U;
    }
    uint32_t pulse_ns = (current_period_ns_ / 100U) * percent;
    return pwm_set_dt(&spec_, current_period_ns_, pulse_ns) == 0;
  }

  void Stop() {
    pwm_set_dt(&spec_, current_period_ns_, 0);
  }

  /**
   * @brief Direct access to Zephyr's native nanosecond API
   * @param period_ns Period in nanoseconds
   * @param pulse_ns Pulse width in nanoseconds
   * @return true on success
   */
  bool SetNs(uint32_t period_ns, uint32_t pulse_ns) {
    current_period_ns_ = period_ns;
    return pwm_set_dt(&spec_, period_ns, pulse_ns) == 0;
  }
};

}  // namespace myl_utils
