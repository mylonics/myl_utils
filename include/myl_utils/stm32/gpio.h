#pragma once

/**
 * @file gpio.h
 * @brief STM32 HAL GPIO implementations
 *
 * Provides GpioOutput / GpioInput / GpioPin implementations backed by
 * the STM32 HAL GPIO driver.
 *
 * @note Requires the STM32 HAL header to be included before this file.
 *       Typically satisfied by including "main.h" or your HAL umbrella
 *       header (e.g. "stm32f4xx_hal.h").
 *
 * @note These classes do NOT call HAL_GPIO_Init() — pin configuration
 *       (mode, pull, speed) is expected to be handled by CubeMX / HAL_GPIO_Init()
 *       before constructing these objects.
 *
 * Usage:
 * @code
 * #include "main.h"  // CubeMX-generated, includes HAL + pin defines
 * #include <myl_utils/stm32/gpio.h>
 *
 * // Use the CubeMX-generated port/pin defines:
 * Stm32GpioOutput led(LED_GPIO_Port, LED_Pin);
 * led.Set(true);
 * led.Toggle();
 *
 * Stm32GpioInput button(BTN_GPIO_Port, BTN_Pin);
 * bool pressed = button.Read();
 * @endcode
 */

#include <myl_utils/gpio.h>

#ifndef HAL_GPIO_MODULE_ENABLED
#error "Include your STM32 HAL header (e.g. stm32f4xx_hal.h) before myl_utils/stm32/gpio.h"
#endif

namespace myl_utils {

/**
 * @brief STM32 HAL GPIO output pin
 */
class Stm32GpioOutput : public GpioOutputBase<Stm32GpioOutput> {
 protected:
  GPIO_TypeDef *port_;
  uint16_t pin_;

 public:
  /**
   * @brief Construct a GPIO output wrapper
   * @param port GPIO port (e.g. GPIOA, LED_GPIO_Port)
   * @param pin GPIO pin mask (e.g. GPIO_PIN_5, LED_Pin)
   * @note The pin must already be configured as an output via HAL_GPIO_Init / CubeMX.
   */
  Stm32GpioOutput(GPIO_TypeDef *port, uint16_t pin)
      : port_(port), pin_(pin) {}

  void Set(bool state) {
    HAL_GPIO_WritePin(port_, pin_, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  void Toggle() {
    HAL_GPIO_TogglePin(port_, pin_);
  }
};

/**
 * @brief STM32 HAL GPIO input pin
 */
class Stm32GpioInput : public GpioInputBase<Stm32GpioInput> {
 protected:
  GPIO_TypeDef *port_;
  uint16_t pin_;

 public:
  /**
   * @brief Construct a GPIO input wrapper
   * @param port GPIO port (e.g. GPIOA, BTN_GPIO_Port)
   * @param pin GPIO pin mask (e.g. GPIO_PIN_0, BTN_Pin)
   * @note The pin must already be configured as an input via HAL_GPIO_Init / CubeMX.
   */
  Stm32GpioInput(GPIO_TypeDef *port, uint16_t pin)
      : port_(port), pin_(pin) {}

  bool Read() {
    return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
  }
};

/**
 * @brief STM32 HAL bidirectional GPIO pin (output + input)
 *
 * Reads the current pin state via the input data register even when
 * configured as an output. Useful for open-drain pins.
 */
class Stm32GpioPin : public GpioPinBase<Stm32GpioPin> {
 protected:
  GPIO_TypeDef *port_;
  uint16_t pin_;

 public:
  /**
   * @brief Construct a bidirectional GPIO wrapper
   * @param port GPIO port (e.g. GPIOA)
   * @param pin GPIO pin mask (e.g. GPIO_PIN_5)
   * @note The pin must already be configured via HAL_GPIO_Init / CubeMX.
   */
  Stm32GpioPin(GPIO_TypeDef *port, uint16_t pin)
      : port_(port), pin_(pin) {}

  void Set(bool state) {
    HAL_GPIO_WritePin(port_, pin_, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  void Toggle() {
    HAL_GPIO_TogglePin(port_, pin_);
  }

  bool Read() {
    return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
  }
};

/**
 * @brief STM32 HAL interrupt-capable GPIO input pin
 *
 * Wraps an EXTI-configured input pin. The pin must be configured for EXTI
 * by CubeMX / HAL_GPIO_Init() before construction.
 *
 * @note STM32 HAL dispatches EXTI interrupts through global callbacks
 *       (HAL_GPIO_EXTI_Callback). This class stores the user callback in a
 *       static table indexed by pin number. Forward the HAL callback:
 * @code
 * void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 *     Stm32GpioInterrupt::HandleIrq(GPIO_Pin);
 * }
 * @endcode
 */
class Stm32GpioInterrupt : public GpioInterruptBase<Stm32GpioInterrupt> {
 protected:
  GPIO_TypeDef *port_;
  uint16_t pin_;

  /// Callback table indexed by pin position (0–15)
  static inline void (*callbacks_[16])(){};

  /// Convert pin mask (GPIO_PIN_x) to index (0–15)
  static uint8_t PinIndex(uint16_t pin) {
    return static_cast<uint8_t>(__builtin_ctz(pin));
  }

 public:
  using Edge = GpioInterruptBase<Stm32GpioInterrupt>::Edge;

  /**
   * @brief Construct an interrupt-capable GPIO input wrapper
   * @param port GPIO port (e.g. GPIOA)
   * @param pin GPIO pin mask (e.g. GPIO_PIN_0)
   * @note The pin must already be configured for EXTI via HAL_GPIO_Init / CubeMX.
   */
  Stm32GpioInterrupt(GPIO_TypeDef *port, uint16_t pin)
      : port_(port), pin_(pin) {}

  bool Read() {
    return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
  }

  bool EnableInterrupt(Edge /*edge*/, void (*callback)()) {
    // Edge configuration is handled by CubeMX / HAL_GPIO_Init.
    // We just register the callback here.
    callbacks_[PinIndex(pin_)] = callback;
    return true;
  }

  void DisableInterrupt() {
    callbacks_[PinIndex(pin_)] = nullptr;
  }

  /**
   * @brief Call from HAL_GPIO_EXTI_Callback to dispatch to the registered handler
   * @param gpio_pin The GPIO_Pin parameter from HAL_GPIO_EXTI_Callback
   */
  static void HandleIrq(uint16_t gpio_pin) {
    uint8_t idx = PinIndex(gpio_pin);
    if (idx < 16 && callbacks_[idx]) {
      callbacks_[idx]();
    }
  }
};

}  // namespace myl_utils
