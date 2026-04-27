#pragma once

/**
 * @file platform.h
 * @brief Zephyr platform convenience header.
 *
 * Includes all available Zephyr peripheral headers (guarded by the
 * corresponding Kconfig macros) and provides unprefixed type aliases inside the
 * myl_utils namespace.  Application code can write:
 *
 *   #include <myl_utils/zephyr/platform.h>
 *   myl_utils::GpioOutput led(...);
 *
 * instead of the longer platform-qualified names.
 *
 * Headers that require a specific Kconfig option are only included (and their
 * aliases only defined) when that option is enabled, so this header is safe to
 * include regardless of which peripherals are configured.
 */

// ===========================================================================
// Always-available headers (no Kconfig guard)
// ===========================================================================
#include <myl_utils/zephyr/gpio.h>
#include <myl_utils/zephyr/pwm.h>
#include <myl_utils/zephyr/timing.h>
#include <myl_utils/zephyr/hw_config.h>
#include <myl_utils/zephyr/hw_helper.h>
#include <myl_utils/zephyr/log.h>
#include <myl_utils/zephyr/work_helper.h>
#include <myl_utils/zephyr/device_id.h>

// ===========================================================================
// Conditionally-available headers
// ===========================================================================

// --- I2C -------------------------------------------------------------------
#ifdef CONFIG_I2C
#include <myl_utils/zephyr/i2c.h>
#endif

// --- SPI -------------------------------------------------------------------
#ifdef CONFIG_SPI
#include <myl_utils/zephyr/spi.h>
#endif

// --- Serial ----------------------------------------------------------------
#ifdef CONFIG_MYL_UTILS_SERIAL
#include <myl_utils/zephyr/serial.h>
#endif

// --- NVM -------------------------------------------------------------------
#ifdef CONFIG_MYL_UTILS_NVM
#include <myl_utils/zephyr/nvm_controller.h>
#endif

// --- ADC -------------------------------------------------------------------
#ifdef CONFIG_ADC
#include <myl_utils/zephyr/adc_helper.h>
#endif

// --- CPU Temperature -------------------------------------------------------
#ifdef CONFIG_MYL_UTILS_CPU_TEMP
#include <myl_utils/zephyr/cpu_temp.h>
#endif

// --- Watchdog --------------------------------------------------------------
#ifdef CONFIG_MYL_UTILS_WATCHDOG
#include <myl_utils/zephyr/watchdog.h>
#endif

// --- BLE -------------------------------------------------------------------
#ifdef CONFIG_BT
#include <myl_utils/zephyr/ble_helper.h>
#include <myl_utils/zephyr/ble_server.h>
#include <myl_utils/zephyr/ble_client.h>
#endif

// --- LoRa ------------------------------------------------------------------
#if defined(CONFIG_LORA) && defined(CONFIG_LORA_MODULE_BACKEND_LORA_BASICS_MODEM)
#include <myl_utils/zephyr/lora_device.h>
#include <myl_utils/zephyr/lora_client.h>
#include <myl_utils/zephyr/lora_server.h>
#endif

// --- USB Serial / USBD -----------------------------------------------------
#ifdef CONFIG_MYL_UTILS_USB_SERIAL
#include <myl_utils/zephyr/usbd_init.h>
#endif

// ===========================================================================
// Unprefixed aliases
// ===========================================================================
namespace myl_utils {

// --- Timing (always available) ---------------------------------------------
using Timing = ZephyrTiming;

// --- GPIO (always available) -----------------------------------------------
using GpioOutput    = ZephyrGpioOutput;
using GpioInput     = ZephyrGpioInput;
using GpioPin       = ZephyrGpioPin;
using GpioInterrupt = ZephyrGpioInterrupt;

// --- PWM (always available) ------------------------------------------------
using PwmOutput = ZephyrPwmOutput;

// --- I2C -------------------------------------------------------------------
#ifdef CONFIG_I2C
using I2cDev = ZephyrI2cTransport;
#endif

// --- SPI -------------------------------------------------------------------
#ifdef CONFIG_SPI
using SpiDev = ZephyrSpiTransport;

#ifdef CONFIG_SPI_ASYNC
template <uint8_t QueueSize = 32>
using AsyncSpiDev = ZephyrAsyncSpiTransport<QueueSize>;
#endif
#endif // CONFIG_SPI

// --- Serial ----------------------------------------------------------------
#ifdef CONFIG_MYL_UTILS_SERIAL
using UartDev = ZephyrUartTransport;

#ifdef CONFIG_MYL_UTILS_UART_ASYNC
template <size_t RxBufSize = 256, size_t ConsumerBufSize = 1024>
using AsyncUartDev = ZephyrAsyncUartTransport<RxBufSize, ConsumerBufSize>;
#endif

#ifdef CONFIG_MYL_UTILS_USB_SERIAL
using UsbSerialDev = ZephyrUsbSerialDevice;
#endif
#endif // CONFIG_MYL_UTILS_SERIAL

// --- NVM -------------------------------------------------------------------
#ifdef CONFIG_MYL_UTILS_NVM
using NvmDev = NvmController;
#endif

} // namespace myl_utils
