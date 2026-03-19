#pragma once

/**
 * @file gpio.h
 * @brief Abstract GPIO pin interface
 *
 * Provides a minimal, platform-agnostic interface for digital GPIO pins.
 * Platform implementations (Zephyr, STM32 HAL, etc.) inherit from the
 * appropriate base class and provide the actual hardware access.
 *
 * Usage:
 * @code
 * // Platform-specific construction (see zephyr/gpio.h or stm32/gpio.h)
 * auto &led = get_led_pin();  // returns GpioOutput&
 *
 * led.Set(true);    // drive high
 * led.Set(false);   // drive low
 * led.Toggle();     // flip state
 *
 * auto &button = get_button_pin();  // returns GpioInput&
 * bool pressed = button.Read();
 *
 * // Interrupt-capable input:
 * auto &irq_pin = get_irq_pin();  // returns GpioInterrupt&
 * irq_pin.EnableInterrupt(GpioInterrupt::Edge::Rising, []() {
 *     // handle interrupt
 * });
 * irq_pin.DisableInterrupt();
 * @endcode
 */

#include <cstdint>

/**
 * @brief Abstract base class for a GPIO output pin
 *
 * Implementations must provide Set() and Toggle().
 */
class GpioOutput {
 public:
  virtual ~GpioOutput() = default;

  /**
   * @brief Drive the pin high or low
   * @param state true = logic high (active), false = logic low (inactive)
   */
  virtual void Set(bool state) = 0;

  /**
   * @brief Toggle the pin state
   */
  virtual void Toggle() = 0;
};

/**
 * @brief Abstract base class for a GPIO input pin
 *
 * Implementations must provide Read().
 */
class GpioInput {
 public:
  virtual ~GpioInput() = default;

  /**
   * @brief Read the current logic level of the pin
   * @return true if the pin is logic high, false if logic low
   */
  virtual bool Read() = 0;
};

/**
 * @brief Abstract base class for a bidirectional GPIO pin
 *
 * Combines both input and output capabilities. Useful for open-drain
 * configurations or pins that switch direction at runtime.
 */
class GpioPin : public GpioOutput, public GpioInput {
 public:
  ~GpioPin() override = default;
};

/**
 * @brief Abstract base class for an interrupt-capable GPIO input pin
 *
 * Extends GpioInput with interrupt enable/disable. The callback is a plain
 * function pointer (no captures) to keep it ISR-safe and zero-overhead.
 */
class GpioInterrupt : public GpioInput {
 public:
  /// Interrupt trigger edge
  enum class Edge {
    Rising,   ///< Trigger on low-to-high transition
    Falling,  ///< Trigger on high-to-low transition
    Both      ///< Trigger on any transition
  };

  ~GpioInterrupt() override = default;

  /**
   * @brief Enable interrupt with the given edge and callback
   * @param edge Which edge(s) to trigger on
   * @param callback Function to call from ISR context when triggered
   * @return true if successfully configured, false on error
   */
  virtual bool EnableInterrupt(Edge edge, void (*callback)()) = 0;

  /**
   * @brief Disable the interrupt on this pin
   */
  virtual void DisableInterrupt() = 0;
};
