#pragma once

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "myl_utils/noncopyable.h"

template <class T, size_t size>
class AdcValues
{
public:
  static const size_t adc_channels_count = size;
  static constexpr float bit_to_volts_conversion = 3.3 / 4096;
  bool valid;

  union
  {
    T data;
    uint16_t raw_buffer[adc_channels_count];
  };
};

#define ADC_DT_SPEC(adc_channel) \
  static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), adc_channel)

#define PREPEND_UINT16(name) \
  uint16_t name

#define ADC_SEQUENCE_DEFINE(sequence_name, ...)                          \
  FOR_EACH(ADC_DT_SPEC, (;), __VA_ARGS__);                               \
  static const struct adc_dt_spec sequence_name[] = {__VA_ARGS__};       \
  static const size_t sequence_name##_count = ARRAY_SIZE(sequence_name); \
  struct sequence_name##_data                                            \
  {                                                                      \
    FOR_EACH(PREPEND_UINT16, (;), __VA_ARGS__);                          \
  };                                                                     \
  typedef AdcValues<sequence_name##_data, sequence_name##_count> sequence_name##Values;

#define ADC_SINGLE_CHANNEL(adc_channel) \
  ADC_DT_SPEC(adc_channel);

#if !defined(BUFFER_MEM_REGION)
#define BUFFER_MEM_REGION EMPTY
#endif
#define ALIGNMENT_VAR 32

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
class AdcHandler : NonCopyable<AdcHandler<T>>
{
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

  bool ConfigureAdc()
  {
    for (size_t i = 0U; i < T::adc_channels_count; i++)
    {
      if (!device_is_ready(adc_channels_[i].dev))
      {
        LOG_MODULE_DECLARE(user, LOG_LEVEL_INF);
        LOG_ERR("ADC controller device %s not ready\n", adc_channels_[i].dev->name);
        return 1;
      }

      if (adc_channel_setup_dt(&adc_channels_[i]))
      {
        LOG_MODULE_DECLARE(user, LOG_LEVEL_INF);
        LOG_ERR("ADC Could not setup channel #%d\n", i);
        return 1;
      }
    }

    if (adc_sequence_init_dt(&adc_channels_[0], &sequence))
    {
      LOG_MODULE_DECLARE(user, LOG_LEVEL_INF);
      LOG_ERR("ADC sequence_init failed\n");
      return 1;
    }

    for (size_t i = 1; i < T::adc_channels_count; i++)
    {
      sequence.channels |= BIT(adc_channels_[i].channel_id);
    }
    k_poll_signal_init(&signal);

    return 0;
  }

public:
  AdcHandler(const adc_dt_spec (&adc_channels)[T::adc_channels_count]) : adc_channels_{adc_channels} { ConfigureAdc(); }

  void Start()
  {
    int ret = adc_read_async(adc_channels_[0].dev, &sequence, &signal);
    if (ret)
    {
      LOG_MODULE_DECLARE(user, LOG_LEVEL_INF);
      LOG_ERR("ADC Read asyc failed %d\n", ret);
    }
  }

  T AdcRead()
  {
    T data{};
    int ret = k_poll(&event, 1, K_NO_WAIT);

    if (ret == 0)
    {
      event.signal->signaled = 0;
      event.state = K_POLL_STATE_NOT_READY;

      for (size_t i = 0; i < T::adc_channels_count; i++)
      {
        data.raw_buffer[i] = adc_hw_buffer[i];
      }

      ret = adc_read_async(adc_channels_[0].dev, &sequence, &signal);
      data.valid = true;
    }
    return data;
  }
};

class AdcSingleChannelHandler : NonCopyable<AdcSingleChannelHandler>
{
public:
  AdcSingleChannelHandler(const struct adc_dt_spec *adc_channel) : adc_channel_{adc_channel}
  {
    LOG_MODULE_DECLARE(user, LOG_LEVEL_INF);
    if (!adc_is_ready_dt(adc_channel_))
    {
      LOG_ERR("ADC controller device %s not ready\n", adc_channel_->dev->name);
    }

    int err = adc_channel_setup_dt(adc_channel_);
    if (err < 0)
    {
      LOG_ERR("Could not setup channel #%d \n", err, adc_channel_->channel_id);
    }
  }

  bool Read(uint16_t &data)
  {
    (void)adc_sequence_init_dt(adc_channel_, &sequence);
    int err = adc_read_dt(adc_channel_, &sequence);
    data = buf;
    return err;
  }

private:
  uint16_t buf;
  const struct adc_dt_spec *adc_channel_;
  struct adc_sequence sequence = {
      .buffer = &buf,
      /* buffer size in bytes, not number of samples */
      .buffer_size = sizeof(buf),
  };
};
