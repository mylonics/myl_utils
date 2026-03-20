#pragma once

/**
 * @file platform.h
 * @brief STM32 platform convenience header.
 *
 * Includes all available STM32 peripheral headers (guarded by the corresponding
 * HAL module macros) and provides unprefixed type aliases inside the myl_utils
 * namespace.  Application code can write:
 *
 *   #include <myl_utils/stm32/platform.h>
 *   myl_utils::GpioOutput led(...);
 *
 * instead of the longer platform-qualified names.
 *
 * @note The STM32 HAL master header (e.g. stm32f4xx_hal.h) must be included
 *       **before** this header so the HAL_*_MODULE_ENABLED macros are defined.
 */

// ---------------------------------------------------------------------------
// Timing (requires the generic HAL umbrella, no specific peripheral module)
// ---------------------------------------------------------------------------
#ifdef HAL_MODULE_ENABLED
#include <myl_utils/stm32/timing.h>
#endif

// ---------------------------------------------------------------------------
// GPIO
// ---------------------------------------------------------------------------
#ifdef HAL_GPIO_MODULE_ENABLED
#include <myl_utils/stm32/gpio.h>
#endif

// ---------------------------------------------------------------------------
// I2C
// ---------------------------------------------------------------------------
#ifdef HAL_I2C_MODULE_ENABLED
#include <myl_utils/stm32/i2c.h>
#endif

// ---------------------------------------------------------------------------
// SPI
// ---------------------------------------------------------------------------
#ifdef HAL_SPI_MODULE_ENABLED
#include <myl_utils/stm32/spi.h>
#endif

// ---------------------------------------------------------------------------
// PWM (uses TIM peripheral)
// ---------------------------------------------------------------------------
#ifdef HAL_TIM_MODULE_ENABLED
#include <myl_utils/stm32/pwm.h>
#include <myl_utils/stm32/timer.h>
#endif

// ===========================================================================
// Unprefixed aliases
// ===========================================================================
namespace myl_utils {

// --- Timing ----------------------------------------------------------------
#ifdef HAL_MODULE_ENABLED
using Timing = Stm32Timing;
#endif

// --- GPIO ------------------------------------------------------------------
#ifdef HAL_GPIO_MODULE_ENABLED
using GpioOutput    = Stm32GpioOutput;
using GpioInput     = Stm32GpioInput;
using GpioPin       = Stm32GpioPin;
using GpioInterrupt = Stm32GpioInterrupt;
#endif

// --- I2C -------------------------------------------------------------------
#ifdef HAL_I2C_MODULE_ENABLED
using I2cDev = Stm32I2cDevice;

template <TransferMode Mode = TransferMode::Interrupt, uint8_t QueueSize = 32>
using AsyncI2cDev = Stm32AsyncI2cDevice<Mode, QueueSize>;

template <uint8_t QueueSize = 32>
using DmaI2cDev = Stm32DmaI2cDevice<QueueSize>;
#endif

// --- SPI -------------------------------------------------------------------
#ifdef HAL_SPI_MODULE_ENABLED
using SpiDev = Stm32SpiDevice;

template <TransferMode Mode = TransferMode::Interrupt, uint8_t QueueSize = 32>
using AsyncSpiDev = Stm32AsyncSpiDevice<Mode, QueueSize>;

template <uint8_t QueueSize = 32>
using DmaSpiDev = Stm32DmaSpiDevice<QueueSize>;
#endif

// --- PWM & Timer -----------------------------------------------------------
#ifdef HAL_TIM_MODULE_ENABLED
using PwmOutput = Stm32PwmOutput;
using HwTimer   = Stm32HwTimer;
#endif

} // namespace myl_utils
