#pragma once

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "myl_utils/noncopyable.h"

#if !defined(BUFFER_MEM_REGION)
#define BUFFER_MEM_REGION EMPTY
#endif
#define ALIGNMENT_VAR 32

bool ConfigureGpios() __attribute__((weak));

/** @brief This class configures the GPIOs. This should be one of the first classes called to ensure that devices start
 * up properly. In order for this class to work, your firmware should define a ConfigureGpios() function.
 */
class GpioConfigure : NonCopyable<GpioConfigure> {
 public:
  GpioConfigure() {
    if (ConfigureGpios) {
      ConfigureGpios();
    }
  }
};

/** @brief This class configures allows for async GPIO read. A class of the following format is required as a templated
 * argument:
 *
 * class AdcValues {
 *  public:
 *   static const size_t adc_channels_count = ARRAY_SIZE(adc_channels);
 *   bool valid;
 *   uint16_t raw_buffer[adc_channels_count];
 *};
 *
 * The class needs to have it Start function called. AdcRead may be periodically called. If the returns valid parameter
 *is true, then new data has been acquired.
 *
 */
template <class T>
class AdcHandler : NonCopyable<AdcHandler<T>> {
 private:
  const adc_dt_spec (&adc_channels_)[T::adc_channels_count];

  struct adc_sequence_options options = {
      /* Start consecutive samplings as fast as possible. */
      .interval_us = 0,
      .extra_samplings = 0,
  };

  BUFFER_MEM_REGION __aligned(ALIGNMENT_VAR) int16_t adc_hw_buffer[T::adc_channels_count * 20];

  struct adc_sequence sequence = {
      .options = &options,
      .buffer = adc_hw_buffer,
      .buffer_size = T::adc_channels_count * 4,
  };

  struct k_poll_signal signal;
  struct k_poll_event event = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &signal);

  bool ConfigureAdc() {
    for (size_t i = 0U; i < T::adc_channels_count; i++) {
      if (!device_is_ready(adc_channels_[i].dev)) {
        LOG_MODULE_DECLARE(user, LOG_LEVEL_INF);
        LOG_ERR("ADC controller device %s not ready\n", adc_channels_[i].dev->name);
        return 1;
      }

      if (adc_channel_setup_dt(&adc_channels_[i])) {
        LOG_MODULE_DECLARE(user, LOG_LEVEL_INF);
        LOG_ERR("ADC Could not setup channel #%d\n", i);
        return 1;
      }
    }

    if (adc_sequence_init_dt(&adc_channels_[0], &sequence)) {
      LOG_MODULE_DECLARE(user, LOG_LEVEL_INF);
      LOG_ERR("ADC sequence_init failed\n");
      return 1;
    }

    for (size_t i = 1; i < T::adc_channels_count; i++) {
      sequence.channels |= BIT(adc_channels_[i].channel_id);
    }
    k_poll_signal_init(&signal);

    return 0;
  }

 public:
  AdcHandler(const adc_dt_spec (&adc_channels)[T::adc_channels_count]) : adc_channels_{adc_channels} { ConfigureAdc(); }

  void Start() {
    int ret = adc_read_async(adc_channels_[0].dev, &sequence, &signal);
    if (ret) {
      LOG_MODULE_DECLARE(user, LOG_LEVEL_INF);
      LOG_ERR("ADC Read asyc failed %d\n", ret);
    }
  }

  T AdcRead() {
    T data{};
    int ret = k_poll(&event, 1, K_NO_WAIT);

    if (ret == 0) {
      event.signal->signaled = 0;
      event.state = K_POLL_STATE_NOT_READY;

      for (size_t i = 0; i < T::adc_channels_count; i++) {
        data.raw_buffer[i] = adc_hw_buffer[i];
      }

      ret = adc_read_async(adc_channels_[0].dev, &sequence, &signal);
      data.valid = true;
    }
    return data;
  }
};
