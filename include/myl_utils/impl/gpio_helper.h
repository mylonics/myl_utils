#pragma once

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#define GPIO_TOGGLE_FUNCTION(gpio) \
  void gpio##_toggle() { gpio_pin_toggle_dt(&gpio); }

#define GPIO_ENABLE_FUNCTION(gpio) \
  void gpio##_enable(bool enable) { gpio_pin_set_dt(&gpio, enable); }

#define GPIO_ENABLE_INV_FUNCTION(gpio) \
  void gpio##_enable(bool enable) { gpio_pin_set_dt(&gpio, !enable); }

#define GPIO_READ(gpio) \
  bool gpio##_read() { return gpio_pin_get_dt(&gpio); }

static inline void configure_and_check_successful(const struct gpio_dt_spec *spec, gpio_flags_t extra_flags) {
  LOG_MODULE_DECLARE(user, LOG_LEVEL_ERR);

  if (gpio_pin_configure_dt(spec, extra_flags)) {
    LOG_ERR("Failed to init gpio %s %d\n", spec->port->name, spec->pin);
  }
}
