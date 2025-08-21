#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#ifndef CONFIG_MYL_UTILS_CPU_TEMP
#error Enable CONFIG_MYL_UTILS_CPU_TEMP to use cpu_temp helper
#endif

// Fetch CPU (die) temperature from nRF TEMP sensor via Zephyr sensor API.
// Returns true on success, false on failure.
static inline bool myl_utils_read_cpu_temp(float &out_celsius) {
  static const struct device *temp_dev = DEVICE_DT_GET_ONE(nordic_nrf_temp);
  static const bool initialized = device_is_ready(temp_dev);
  if (!initialized) {
    return false;
  }

  if (sensor_sample_fetch(temp_dev) != 0) {
    return false;
  }
  struct sensor_value val;
  if (sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP, &val) != 0) {
    return false;
  }
  out_celsius = val.val1 + val.val2 / 1000000.0f;
  return true;
}
