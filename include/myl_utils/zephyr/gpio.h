#pragma once

/**
 * @file gpio.h
 * @brief Zephyr GPIO implementations
 *
 * Provides GpioOutput / GpioInput / GpioPin / GpioInterrupt implementations
 * backed by Zephyr's gpio_dt_spec API.
 *
 * Usage:
 * @code
 * // From devicetree:
 * //   leds { compatible = "gpio-leds"; led0: led_0 { gpios = <&gpio0 13 GPIO_ACTIVE_LOW>; }; };
 * static const struct gpio_dt_spec led_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);
 * ZephyrGpioOutput<> led(led_spec, true);       // normal polarity
 * ZephyrGpioOutput<true> led_inv(led_spec, true); // inverted polarity
 *
 * led.Set(true);
 * led.Toggle();
 *
 * static const struct gpio_dt_spec btn_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(button0), gpios);
 * ZephyrGpioInterrupt<> button(btn_spec);
 * ZephyrGpioInterrupt<true> button_inv(btn_spec); // Read() / edges are logically inverted
 * bool pressed = button.Read();
 * button.EnableInterrupt(ZephyrGpioInterrupt<>::Edge::Falling, my_handler);
 * @endcode
 */

#include <myl_utils/gpio.h>
#include <zephyr/drivers/gpio.h>

namespace myl_utils {

/**
 * @brief Zephyr GPIO output pin
 *
 * Wraps a gpio_dt_spec and configures the pin as an output on construction.
 *
 * @tparam Invert When true, logical Set(true) drives the pin inactive and
 *                Set(false) drives it active. Resolved entirely at compile
 *                time — zero runtime overhead.
 */
template <bool Invert = false>
class ZephyrGpioOutput : public GpioOutputBase<ZephyrGpioOutput<Invert>>,
                         NonCopyable<ZephyrGpioOutput<Invert>> {
 protected:
  const struct gpio_dt_spec spec_;

 public:

  /**
   * @brief Construct and configure a GPIO output
   * @param spec Devicetree GPIO spec (from GPIO_DT_SPEC_GET)
   * @param initial_state Initial logic level (true = active, false = inactive)
   */
  ZephyrGpioOutput(const struct gpio_dt_spec &spec, bool initial_state = false)
      : spec_(spec) {
    const bool physical = Invert ? !initial_state : initial_state;
    gpio_pin_configure_dt(&spec_, physical ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE);
  }

  MYL_NOINLINE void Set(bool state) {
    gpio_pin_set_dt(&spec_, Invert ? !state : state);
  }

  MYL_NOINLINE void Toggle() {
    gpio_pin_toggle_dt(&spec_);
  }
};

/**
 * @brief Zephyr GPIO input pin
 *
 * Wraps a gpio_dt_spec and configures the pin as an input on construction.
 *
 * @tparam Invert When true, Read() returns the logical complement of the
 *                physical pin level. Resolved at compile time.
 */
template <bool Invert = false>
class ZephyrGpioInput : public GpioInputBase<ZephyrGpioInput<Invert>>,
                        NonCopyable<ZephyrGpioInput<Invert>> {
 protected:
  const struct gpio_dt_spec spec_;

 public:

  /**
   * @brief Construct and configure a GPIO input
   * @param spec Devicetree GPIO spec (from GPIO_DT_SPEC_GET)
   */
  explicit ZephyrGpioInput(const struct gpio_dt_spec &spec)
      : spec_(spec) {
    gpio_pin_configure_dt(&spec_, GPIO_INPUT);
  }

  MYL_NOINLINE bool Read() {
    return (gpio_pin_get_dt(&spec_) != 0) ^ Invert;
  }
};

/**
 * @brief Zephyr bidirectional GPIO pin (output + input)
 *
 * Configured as output by default. Read() returns the current driven/sensed level.
 * Useful for open-drain pins or pins that need both read and write access.
 *
 * @tparam Invert When true, Set() and Read() operate on the logical complement
 *                of the physical level. Resolved at compile time.
 */
template <bool Invert = false>
class ZephyrGpioPin : public GpioPinBase<ZephyrGpioPin<Invert>>,
                      NonCopyable<ZephyrGpioPin<Invert>> {
 protected:
  const struct gpio_dt_spec spec_;

 public:

  /**
   * @brief Construct and configure a bidirectional GPIO pin
   * @param spec Devicetree GPIO spec (from GPIO_DT_SPEC_GET)
   * @param initial_state Initial logic level (true = active, false = inactive)
   */
  ZephyrGpioPin(const struct gpio_dt_spec &spec, bool initial_state = false)
      : spec_(spec) {
    const bool physical = Invert ? !initial_state : initial_state;
    gpio_pin_configure_dt(&spec_,
        (physical ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE) | GPIO_INPUT);
  }

  MYL_NOINLINE void Set(bool state) {
    gpio_pin_set_dt(&spec_, Invert ? !state : state);
  }

  MYL_NOINLINE void Toggle() {
    gpio_pin_toggle_dt(&spec_);
  }

  MYL_NOINLINE bool Read() {
    return (gpio_pin_get_dt(&spec_) != 0) ^ Invert;
  }
};

/**
 * @brief Zephyr interrupt-capable GPIO input pin
 *
 * Configures the pin as an input and provides interrupt enable/disable.
 * The user-supplied callback is stored and invoked from the Zephyr GPIO
 * ISR via a static trampoline.
 *
 * @tparam Invert When true, Read() returns the logical complement and
 *                Rising/Falling edges are swapped in EnableInterrupt().
 *                Resolved at compile time.
 */
template <bool Invert = false>
class ZephyrGpioInterrupt : public GpioInterruptBase<ZephyrGpioInterrupt<Invert>>,
                            NonCopyable<ZephyrGpioInterrupt<Invert>> {
 protected:
  const struct gpio_dt_spec spec_;
  struct gpio_callback cb_data_{};
  void (*user_callback_)(){};

  /// Trampoline: Zephyr calls this; we recover the object and invoke the user callback
  static void IsrTrampoline(const struct device * /*dev*/, struct gpio_callback *cb, uint32_t /*pins*/) {
    auto *self = CONTAINER_OF(cb, ZephyrGpioInterrupt<Invert>, cb_data_);
    if (self->user_callback_) {
      self->user_callback_();
    }
  }

 public:
  using Edge = GpioInterruptBase<ZephyrGpioInterrupt<Invert>>::Edge;

  /**
   * @brief Construct and configure an interrupt-capable GPIO input
   * @param spec Devicetree GPIO spec (from GPIO_DT_SPEC_GET)
   */
  explicit ZephyrGpioInterrupt(const struct gpio_dt_spec &spec)
      : spec_(spec) {
    gpio_pin_configure_dt(&spec_, GPIO_INPUT);
  }

  MYL_NOINLINE bool Read() {
    return (gpio_pin_get_dt(&spec_) != 0) ^ Invert;
  }

  MYL_NOINLINE bool EnableInterrupt(Edge edge, void (*callback)()) {
    user_callback_ = callback;

    gpio_flags_t zephyr_edge{};
    // When Invert=true, Rising logical edge corresponds to a physical Falling
    // edge and vice versa. The ternary is resolved at compile time.
    switch (edge) {
      case Edge::Rising:  zephyr_edge = Invert ? GPIO_INT_EDGE_TO_INACTIVE : GPIO_INT_EDGE_TO_ACTIVE;   break;
      case Edge::Falling: zephyr_edge = Invert ? GPIO_INT_EDGE_TO_ACTIVE   : GPIO_INT_EDGE_TO_INACTIVE; break;
      case Edge::Both:    zephyr_edge = GPIO_INT_EDGE_BOTH; break;
    }

    int ret = gpio_pin_interrupt_configure_dt(&spec_, zephyr_edge);
    if (ret != 0) return false;

    gpio_init_callback(&cb_data_, IsrTrampoline, BIT(spec_.pin));
    gpio_add_callback(spec_.port, &cb_data_);
    return true;
  }

  MYL_NOINLINE void DisableInterrupt() {
    gpio_pin_interrupt_configure_dt(&spec_, GPIO_INT_DISABLE);
    gpio_remove_callback(spec_.port, &cb_data_);
    user_callback_ = nullptr;
  }
};

}  // namespace myl_utils
