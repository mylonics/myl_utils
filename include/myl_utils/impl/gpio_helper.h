#pragma once

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#define DECLARE_PWM(pwm) \
  static const struct pwm_dt_spec pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm));

#define DECLARE_GPIO(gpio) \
  static const struct gpio_dt_spec gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(gpio), gpios);

#define DECLARE_DEV(dev_name) \
  static const struct device *dev_name##_dev = DEVICE_DT_GET(DT_NODELABEL(dev_name));

#define INIT_PWM(pwm)         \
  if (!pwm_is_ready_dt(&pwm)) \
    return 1;

static inline bool configure_and_check_successful(const struct gpio_dt_spec *spec, gpio_flags_t extra_flags)
{
  LOG_MODULE_DECLARE(user, LOG_LEVEL_ERR);

  if (gpio_pin_configure_dt(spec, extra_flags))
  {
    LOG_ERR("Failed to init gpio %s %d\n", spec->port->name, spec->pin);
    return 1;
  }
  return 0;
}

#define INIT_GPIO_OUTPUT_ACTIVE(gpio)                            \
  if (!gpio_is_ready_dt(&gpio))                                  \
    return 1;                                                    \
  if (configure_and_check_successful(&gpio, GPIO_OUTPUT_ACTIVE)) \
    return 1;

#define INIT_GPIO_OUTPUT_INACTIVE(gpio)                            \
  if (!gpio_is_ready_dt(&gpio))                                    \
    return 1;                                                      \
  if (configure_and_check_successful(&gpio, GPIO_OUTPUT_INACTIVE)) \
    return 1;

#define INIT_GPIO_INPUT(gpio)                            \
  if (!gpio_is_ready_dt(&gpio))                          \
    return 1;                                            \
  if (configure_and_check_successful(&gpio, GPIO_INPUT)) \
    return 1;

#define GPIO_TOGGLE_FUNCTION(gpio) \
  void gpio##_toggle() { gpio_pin_toggle_dt(&gpio); }

#define GPIO_ENABLE_FUNCTION(gpio) \
  void gpio##_enable(bool enable) { gpio_pin_set_dt(&gpio, enable); }

#define GPIO_ENABLE_INV_FUNCTION(gpio) \
  void gpio##_enable(bool enable) { gpio_pin_set_dt(&gpio, !enable); }

#define GPIO_READ(gpio) \
  bool gpio##_read() { return gpio_pin_get_dt(&gpio); }
