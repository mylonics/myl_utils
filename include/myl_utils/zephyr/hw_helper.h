#pragma once

#include <zephyr/drivers/gpio.h>

#include "log.h"

#define DECLARE_PWM(pwm) static const struct pwm_dt_spec pwm = PWM_DT_SPEC_GET(DT_NODELABEL(pwm));

#define DECLARE_GPIO(gpio) static const struct gpio_dt_spec gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(gpio), gpios);

#define DECLARE_DEV(dev_name) static const struct device *dev_name##_dev = DEVICE_DT_GET(DT_NODELABEL(dev_name));

#define INIT_PWM(pwm) \
  if (!pwm_is_ready_dt(&pwm)) return 1;

static inline bool configure_and_check_successful(const struct gpio_dt_spec *spec, gpio_flags_t extra_flags) {
  DECLARE_MYL_UTILS_LOG();

  if (gpio_pin_configure_dt(spec, extra_flags)) {
    LOG_ERR("Failed to init gpio %s %d\n", spec->port->name, spec->pin);
    return 1;
  }
  return 0;
}

#define INIT_GPIO_OUTPUT_ACTIVE(gpio)     \
  if (!gpio_is_ready_dt(&gpio)) return 1; \
  if (configure_and_check_successful(&gpio, GPIO_OUTPUT_ACTIVE)) return 1;

#define INIT_GPIO_OUTPUT_INACTIVE(gpio)   \
  if (!gpio_is_ready_dt(&gpio)) return 1; \
  if (configure_and_check_successful(&gpio, GPIO_OUTPUT_INACTIVE)) return 1;

#define INIT_GPIO_INPUT(gpio)             \
  if (!gpio_is_ready_dt(&gpio)) return 1; \
  if (configure_and_check_successful(&gpio, GPIO_INPUT)) return 1;

#define GPIO_INTERUPT_CB_HELPER(gpio, handler) \
  static struct gpio_callback gpio##_cb_data;  \
  void gpio##_cb_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { handler(); }

#define INIT_GPIO_INTERUPT_WITH_HANDLER(gpio, handler, edge)                                                  \
  int gpio##_ret = gpio_pin_interrupt_configure_dt(&gpio, edge);                                              \
  if (gpio##_ret != 0) {                                                                                      \
    DECLARE_MYL_UTILS_LOG();                                                                                  \
    LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n", gpio##_ret, gpio.port->name, gpio.pin); \
    return 1;                                                                                                 \
  }                                                                                                           \
  gpio_init_callback(&gpio##_cb_data, handler, BIT(gpio.pin));                                                \
  gpio_add_callback(gpio.port, &gpio##_cb_data);

#define INIT_GPIO_INTERUPT(gpio, edge) INIT_GPIO_INTERUPT_WITH_HANDLER(gpio, gpio##_cb_handler, edge)

#define GPIO_TOGGLE_FUNCTION(gpio) \
  void gpio##_toggle() { gpio_pin_toggle_dt(&gpio); }

#define GPIO_ENABLE_FUNCTION(gpio) \
  void gpio##_enable(bool enable) { gpio_pin_set_dt(&gpio, enable); }

#define GPIO_ENABLE_INV_FUNCTION(gpio) \
  void gpio##_enable(bool enable) { gpio_pin_set_dt(&gpio, !enable); }

#define GPIO_READ(gpio) \
  bool gpio##_read() { return gpio_pin_get_dt(&gpio); }
