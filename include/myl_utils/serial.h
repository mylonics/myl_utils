#pragma once

/**
 * @file serial.h
 * @brief CRTP base class for UART/LPUART transports (zero virtual overhead)
 *
 * Provides UartBase<Derived> — a CRTP base that dispatches all stream
 * operations to the concrete transport (Stm32UartTransport, ZephyrUartTransport,
 * ZephyrAsyncUartTransport, etc.) at compile time.
 *
 * Usage:
 * @code
 * // STM32 interrupt-driven UART
 * Stm32UartTransport<> uart(&huart1);
 * uart.Start();
 * uart.PutArray(buf, len);
 *
 * // Zephyr interrupt-driven UART
 * ZephyrUartTransport uart(DEVICE_DT_GET(DT_CHOSEN(zephyr_console)));
 * uart.Start();
 * char c; uart.GetC(c, 100);
 * @endcode
 *
 * Concrete transport requirements (non-virtual methods in Derived):
 *   bool     ImplStart()
 *   void     ImplReconfigureUart(UartBaud, UartParity, UartStopBits)
 *   void     ImplPutC(char)
 *   void     ImplPutArray(uint8_t *, size_t)
 *   bool     ImplReadable()
 *   char     ImplGetC()
 *   bool     ImplGetCTimeout(char &, size_t timeout_ms)
 *   size_t   ImplGetArray(uint8_t *, size_t max_length)
 */

#include "buffer.h"
#include "config.h"

namespace myl_utils {

// ---------------------------------------------------------------------------
// UART configuration enums
// ---------------------------------------------------------------------------

enum class UartBaud {
  B_1200 = 0,
  B_2400 = 1,
  B_4800 = 2,
  B_9600 = 3,
  B_14400 = 4,
  B_19200 = 5,
  B_28800 = 6,
  B_38400 = 7,
  B_57600 = 8,
  B_76800 = 9,
  B_115200 = 10,
  B_END
};

inline size_t UartBaudEnumToValue(UartBaud baud) {
  switch (baud) {
    case UartBaud::B_1200:   return 1200;
    case UartBaud::B_2400:   return 2400;
    case UartBaud::B_4800:   return 4800;
    case UartBaud::B_14400:  return 14400;
    case UartBaud::B_19200:  return 19200;
    case UartBaud::B_28800:  return 28800;
    case UartBaud::B_38400:  return 38400;
    case UartBaud::B_57600:  return 57600;
    case UartBaud::B_76800:  return 76800;
    case UartBaud::B_115200: return 115200;
    case UartBaud::B_9600:
    default:                 return 9600;
  }
}

enum class UartStopBits { ONE = 0, TWO = 1 };

enum class UartParity { NONE = 0, EVEN = 1, ODD = 2 };

enum class UartDataBits {
  FIVE = 0,   ///< 5 data bits
  SIX = 1,    ///< 6 data bits
  SEVEN = 2,  ///< 7 data bits
  EIGHT = 3,  ///< 8 data bits
  NINE = 4,   ///< 9 data bits
};

// ---------------------------------------------------------------------------
// UartBase<Derived> — CRTP base, zero virtual overhead
// ---------------------------------------------------------------------------

/**
 * @brief CRTP base for all UART/LPUART transports
 *
 * All public methods dispatch to Derived::Impl*() at compile time.
 * No vtable, no heap allocation, no runtime dispatch.
 *
 * @tparam Derived Concrete transport class (Stm32UartTransport, ZephyrUartTransport, …)
 */
template <class Derived>
class UartBase {
 public:
  /// Initialize hardware and start reception. Must be called before any I/O.
  MYL_NOINLINE bool Start() { return derived().ImplStart(); }

  /// Change baud rate, parity and stop bits at runtime.
  MYL_NOINLINE void ReconfigureUart(UartBaud baud, UartParity parity, UartStopBits stop_bits) {
    derived().ImplReconfigureUart(baud, parity, stop_bits);
  }

  /// Transmit a single character.
  MYL_NOINLINE void PutC(char c) { derived().ImplPutC(c); }

  /// Transmit an array of bytes.
  MYL_NOINLINE void PutArray(uint8_t *data, size_t size) { derived().ImplPutArray(data, size); }

  /// Return true if at least one received byte is available.
  MYL_NOINLINE bool Readable() { return derived().ImplReadable(); }

  /// Read one byte (blocking until a byte is available in interrupt/DMA mode,
  /// or until RXNE in polling mode).
  MYL_NOINLINE char GetC() { return derived().ImplGetC(); }

  /// Read one byte with a timeout. Returns true if a byte was received within
  /// @p timeout_ms milliseconds.
  MYL_NOINLINE bool GetC(char &c, size_t timeout_ms) { return derived().ImplGetCTimeout(c, timeout_ms); }

  /// Read up to @p max_length bytes into @p data. Returns the number of bytes
  /// actually read (may be less if the RX buffer has fewer bytes available).
  MYL_NOINLINE size_t GetArray(uint8_t *data, size_t max_length) {
    return derived().ImplGetArray(data, max_length);
  }

 private:
  Derived &derived() { return *static_cast<Derived *>(this); }
  const Derived &derived() const { return *static_cast<const Derived *>(this); }
};

// ---------------------------------------------------------------------------
// SerialRepeater — forwards bytes between two arbitrary UART transports
// ---------------------------------------------------------------------------

/**
 * @brief Bidirectional byte forwarder between two UartBase-derived transports.
 *
 * Call Runner() periodically (e.g. from a task or main loop) to drain each
 * transport's RX buffer into the other's TX path.
 *
 * @tparam Uart1 First transport (must be UartBase<Uart1>)
 * @tparam Uart2 Second transport (must be UartBase<Uart2>)
 */
template <class Uart1, class Uart2>
class SerialRepeater {
 public:
  SerialRepeater(Uart1 &u1, Uart2 &u2) : u1_{u1}, u2_{u2} {}

  void Runner() {
    while (u1_.Readable()) { u2_.PutC(u1_.GetC()); }
    while (u2_.Readable()) { u1_.PutC(u2_.GetC()); }
  }

 private:
  Uart1 &u1_;
  Uart2 &u2_;
};

}  // namespace myl_utils