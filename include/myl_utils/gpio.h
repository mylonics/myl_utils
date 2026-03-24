#pragma once

/**
 * @file gpio.h
 * @brief Abstract GPIO pin interface (CRTP — zero virtual overhead)
 *
 * Provides a minimal, platform-agnostic interface for digital GPIO pins using
 * the Curiously Recurring Template Pattern (CRTP). All calls are resolved at
 * compile time — no vtable, no indirect branches.
 *
 * Platform implementations (Zephyr, STM32 HAL, etc.) inherit from the
 * appropriate base class template parametrized on themselves.
 *
 * Usage:
 * @code
 * // Platform-specific construction (see zephyr/gpio.h or stm32/gpio.h)
 * auto &led = get_led_pin();  // returns concrete type
 *
 * led.Set(true);    // drive high — fully inlined
 * led.Set(false);   // drive low
 * led.Toggle();     // flip state
 *
 * auto &button = get_button_pin();
 * bool pressed = button.Read();
 *
 * // Generic code uses templates (zero-cost):
 * template <typename Gpio>
 * void blink(Gpio &led) { led.Toggle(); }
 * @endcode
 */

#include "config.h"
#include <cstdint>
#include <type_traits>

namespace myl_utils {

/**
 * @brief CRTP base class for a GPIO output pin
 *
 * Implementations must provide Set(bool) and Toggle() as public non-virtual methods.
 * The CRTP base forwards calls to the derived class at compile time.
 *
 * @tparam Derived The concrete implementation class (CRTP parameter)
 */
template <typename Derived>
class GpioOutputBase {
 public:
  /**
   * @brief Drive the pin high or low
   * @param state true = logic high (active), false = logic low (inactive)
   */
  MYL_NOINLINE void Set(bool state) { derived().Set(state); }

  /**
   * @brief Toggle the pin state
   */
  MYL_NOINLINE void Toggle() { derived().Toggle(); }

 protected:
  GpioOutputBase() = default;
  ~GpioOutputBase() = default;

 private:
  Derived &derived() { return static_cast<Derived &>(*this); }
  const Derived &derived() const { return static_cast<const Derived &>(*this); }
};

/**
 * @brief CRTP base class for a GPIO input pin
 *
 * Implementations must provide Read() as a public non-virtual method.
 *
 * @tparam Derived The concrete implementation class (CRTP parameter)
 */
template <typename Derived>
class GpioInputBase {
 public:
  /**
   * @brief Read the current logic level of the pin
   * @return true if the pin is logic high, false if logic low
   */
  MYL_NOINLINE bool Read() { return derived().Read(); }

 protected:
  GpioInputBase() = default;
  ~GpioInputBase() = default;

 private:
  Derived &derived() { return static_cast<Derived &>(*this); }
};

/**
 * @brief CRTP base class for a bidirectional GPIO pin
 *
 * Combines both input and output capabilities. Useful for open-drain
 * configurations or pins that switch direction at runtime.
 *
 * @tparam Derived The concrete implementation class (CRTP parameter)
 */
template <typename Derived>
class GpioPinBase : public GpioOutputBase<Derived>, public GpioInputBase<Derived> {
 protected:
  GpioPinBase() = default;
  ~GpioPinBase() = default;
};

/**
 * @brief CRTP base class for an interrupt-capable GPIO input pin
 *
 * Extends GpioInputBase with interrupt enable/disable. The callback is a plain
 * function pointer (no captures) to keep it ISR-safe and zero-overhead.
 *
 * @tparam Derived The concrete implementation class (CRTP parameter)
 */
template <typename Derived>
class GpioInterruptBase : public GpioInputBase<Derived> {
 public:
  /// Interrupt trigger edge
  enum class Edge {
    Rising,   ///< Trigger on low-to-high transition
    Falling,  ///< Trigger on high-to-low transition
    Both      ///< Trigger on any transition
  };

  /**
   * @brief Enable interrupt with the given edge and callback
   * @param edge Which edge(s) to trigger on
   * @param callback Function to call from ISR context when triggered
   * @return true if successfully configured, false on error
   */
  MYL_NOINLINE bool EnableInterrupt(Edge edge, void (*callback)()) {
    return derived().EnableInterrupt(edge, callback);
  }

  /**
   * @brief Disable the interrupt on this pin
   */
  MYL_NOINLINE void DisableInterrupt() { derived().DisableInterrupt(); }

 protected:
  GpioInterruptBase() = default;
  ~GpioInterruptBase() = default;

 private:
  Derived &derived() { return static_cast<Derived &>(*this); }
};

/**
 * @brief Lightweight type-erased GPIO output for chip-select storage
 *
 * Used where a non-template pointer to "any GPIO output" is needed (e.g. SpiPacket).
 * Stores a void* instance pointer and a function pointer — no vtable, single
 * indirect call. Constructed from any type that has a Set(bool) method.
 *
 * The @p active_low parameter (default: true) controls the physical pin polarity
 * when chip-select is activated. With active_low=true (standard SPI convention),
 * Set(true) drives the pin LOW (selecting the device) and Set(false) drives it HIGH.
 * This is encoded in the function-pointer trampoline — zero extra storage or branching.
 *
 * @note When using Zephyr's gpio_dt_spec with GPIO_ACTIVE_LOW, the GPIO driver
 *       already handles inversion. In that case pass active_low=false to avoid
 *       double inversion.
 *
 * Cost: 2 pointer loads + 1 indirect call (same or better than virtual dispatch).
 * Size: 8 bytes on 32-bit (vs. 4 bytes for a raw pointer, but no vtable needed).
 */
struct ChipSelectPin {
  /// No-op function used as default (eliminates branch on every Set() call)
  static void NoOp(void *, bool) {}

  void *instance{};
  void (*set_fn)(void *, bool){NoOp};

  ChipSelectPin() = default;

  /// Construct from any GPIO type that has a Set(bool) method
  /// @param gpio The GPIO output pin to drive
  /// @param active_low If true (default), Set(true) drives the pin LOW
  ///                   (standard SPI chip-select convention).
  ///                   If false, Set(true) drives the pin HIGH.
  template <typename Gpio,
            typename = std::enable_if_t<
                !std::is_same_v<std::decay_t<Gpio>, ChipSelectPin>>>
  explicit ChipSelectPin(Gpio &gpio, bool active_low = true)
      : instance(&gpio),
        set_fn(active_low ? &TrampolineActiveLow<Gpio> : &Trampoline<Gpio>) {}

  MYL_NOINLINE void Set(bool state) const { set_fn(instance, state); }

  explicit operator bool() const { return set_fn != NoOp; }

 private:
  /// Named trampoline — passes state through unchanged (active-high CS).
  template <typename Gpio>
  MYL_NOINLINE static void Trampoline(void *p, bool state) {
    static_cast<Gpio *>(p)->Set(state);
  }

  /// Inverted trampoline — inverts state before calling the GPIO (active-low CS).
  template <typename Gpio>
  MYL_NOINLINE static void TrampolineActiveLow(void *p, bool state) {
    static_cast<Gpio *>(p)->Set(!state);
  }
};

}  // namespace myl_utils
