#pragma once

/**
 * @file hw_helper.h
 * @brief Zephyr devicetree declaration and PWM helper macros
 *
 * For GPIO read/write/toggle, use the class-based interface in
 * myl_utils/zephyr/gpio.h (ZephyrGpioOutput, ZephyrGpioInput, etc.)
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "log.h"

#define DECLARE_PWM(pwm) static const struct pwm_dt_spec pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm));

#define DECLARE_GPIO(gpio) static const struct gpio_dt_spec gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(gpio), gpios);

#define DECLARE_DEV(dev_name) static const struct device *dev_name##_dev = DEVICE_DT_GET(DT_NODELABEL(dev_name));

#define INIT_PWM(pwm) \
  if (!pwm_is_ready_dt(&pwm)) return 1;
