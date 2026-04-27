#pragma once

/**
 * @file stm32/uart.h
 * @brief STM32 HAL UART/LPUART transport (polling, interrupt, DMA)
 *
 * Provides Stm32UartTransport<Mode, RxBufSize, TxBufSize> — a CRTP-based
 * transport that wraps STM32 HAL UART functions for use with UartBase<Derived>.
 *
 * Works with any UART or LPUART peripheral — pass the appropriate handle:
 *   UART_HandleTypeDef huart1;   // UART1
 *   UART_HandleTypeDef hlpuart1; // LPUART1
 * The HAL API is identical for both.
 *
 * Transfer modes (selected at compile time via template parameter):
 *   TransferMode::Polling   — HAL_UART_Transmit/Receive (blocking, no ISR)
 *   TransferMode::Interrupt — HAL_UART_Transmit_IT/Receive_IT (ISR-driven)
 *   TransferMode::Dma       — HAL_UART_Transmit_DMA + HAL_UART_Receive_IT (DMA TX, IT RX)
 *
 * ISR integration (Interrupt and Dma modes):
 *   Call uart.TxCmpltCb() from HAL_UART_TxCpltCallback()
 *   Call uart.RxCmpltCb() from HAL_UART_RxCpltCallback()
 *
 * @note RxBufSize and TxBufSize must be powers of 2 (CircularBuffer requirement).
 * @note DMA TX buffers must be in DMA-accessible RAM on STM32 F7/H7 (not DTCM/ITCM).
 *
 * Usage (interrupt mode):
 * @code
 * // main.h or stm32xx_hal.h included before this
 * #include <myl_utils/stm32/uart.h>
 *
 * extern UART_HandleTypeDef huart1;
 * myl_utils::Stm32UartTransport<> uart(&huart1);
 *
 * int main() {
 *   uart.Start();
 *   uart.PutArray(reinterpret_cast<uint8_t*>("Hello\r\n"), 7);
 * }
 *
 * void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h) { if (h == &huart1) uart.TxCmpltCb(); }
 * void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) { if (h == &huart1) uart.RxCmpltCb(); }
 * @endcode
 *
 * Usage (polling mode, no ISR callbacks needed):
 * @code
 * myl_utils::Stm32UartTransport<myl_utils::TransferMode::Polling> uart(&huart1);
 * uart.Start();
 * uart.PutArray(reinterpret_cast<uint8_t*>("Hello\r\n"), 7);
 * char c; uart.GetC(c, 1000); // blocks up to 1 s
 * @endcode
 *
 * LPUART alias example:
 * @code
 * extern UART_HandleTypeDef hlpuart1;
 * myl_utils::Stm32UartTransport<> lpuart(&hlpuart1); // identical API
 * @endcode
 */

// Requires STM32 HAL to be included before this header (e.g. stm32f4xx_hal.h).
#ifndef HAL_UART_MODULE_ENABLED
#error "Include your STM32 HAL header (e.g. stm32f4xx_hal.h) before myl_utils/stm32/uart.h"
#endif

#include <myl_utils/peripheral.h>  // TransferMode enum
#include <myl_utils/serial.h>      // UartBase<Derived>, enums
#include <myl_utils/noncopyable.h>

namespace myl_utils {

/**
 * @brief STM32 HAL UART/LPUART transport
 *
 * Inherits UartBase<Self> and implements all Impl* methods using STM32 HAL.
 *
 * @tparam Mode        Transfer mode: Polling, Interrupt (default), or Dma
 * @tparam RxBufSize   RX ring buffer size in bytes (power of 2, used in Interrupt/Dma mode)
 * @tparam TxBufSize   TX ring buffer + staging buffer size in bytes (power of 2, used in Interrupt/Dma mode)
 */
template <TransferMode Mode = TransferMode::Interrupt,
          size_t RxBufSize = 256,
          size_t TxBufSize = 256>
class Stm32UartTransport
    : public UartBase<Stm32UartTransport<Mode, RxBufSize, TxBufSize>>,
      NonCopyable<Stm32UartTransport<Mode, RxBufSize, TxBufSize>> {
  friend class UartBase<Stm32UartTransport<Mode, RxBufSize, TxBufSize>>;

  static_assert(Mode == TransferMode::Polling ||
                    ((RxBufSize & (RxBufSize - 1)) == 0 && (TxBufSize & (TxBufSize - 1)) == 0),
                "RxBufSize and TxBufSize must be powers of 2 for Interrupt/Dma mode");

 public:
  /**
   * @param huart      Pointer to HAL UART (or LPUART) handle
   * @param timeout_ms Timeout in milliseconds for polling-mode transfers (ignored in IT/DMA modes)
   */
  explicit Stm32UartTransport(UART_HandleTypeDef *huart, uint32_t timeout_ms = 100)
      : huart_(huart), timeout_(timeout_ms),
        rx_producer_(rx_buf_.MakeProducer()), rx_consumer_(rx_buf_.MakeConsumer()),
        tx_producer_(tx_buf_.MakeProducer()), tx_consumer_(tx_buf_.MakeConsumer()) {}

  // ---- ISR integration (Interrupt / Dma modes only) --------------------

  /**
   * @brief Call from HAL_UART_TxCpltCallback() to continue queued TX.
   * No-op in polling mode.
   */
  MYL_ISR_NOINLINE void TxCmpltCb() {
    if constexpr (Mode != TransferMode::Polling) {
      StartTx();
    }
  }

  /**
   * @brief Call from HAL_UART_RxCpltCallback() to store the received byte and re-arm RX.
   * No-op in polling mode.
   */
  MYL_ISR_NOINLINE void RxCmpltCb() {
    if constexpr (Mode != TransferMode::Polling) {
      rx_producer_.Put(rx_byte_);
      HAL_UART_Receive_IT(huart_, &rx_byte_, 1);
    }
  }

  HAL_StatusTypeDef LastStatus() const { return last_status_; }

 protected:
  // ---- UartBase<Derived> interface -------------------------------------

  MYL_NOINLINE bool ImplStart() {
    if constexpr (Mode != TransferMode::Polling) {
      // Prime single-byte interrupt-driven RX
      last_status_ = HAL_UART_Receive_IT(huart_, &rx_byte_, 1);
      return last_status_ == HAL_OK;
    }
    return true;
  }

  MYL_NOINLINE void ImplReconfigureUart(UartBaud baud, UartParity parity, UartStopBits stop_bits) {
    if constexpr (Mode != TransferMode::Polling) {
      // Disable interrupts during reconfiguration
      HAL_UART_AbortReceive_IT(huart_);
    }

    huart_->Init.BaudRate = static_cast<uint32_t>(UartBaudEnumToValue(baud));

    switch (parity) {
      case UartParity::EVEN: huart_->Init.Parity = UART_PARITY_EVEN; break;
      case UartParity::ODD:  huart_->Init.Parity = UART_PARITY_ODD;  break;
      default:               huart_->Init.Parity = UART_PARITY_NONE; break;
    }
    huart_->Init.StopBits = (stop_bits == UartStopBits::TWO) ? UART_STOPBITS_2 : UART_STOPBITS_1;
    HAL_UART_Init(huart_);

    if constexpr (Mode != TransferMode::Polling) {
      HAL_UART_Receive_IT(huart_, &rx_byte_, 1);
    }
  }

  MYL_NOINLINE void ImplPutC(char c) {
    ImplPutArray(reinterpret_cast<uint8_t *>(&c), 1);
  }

  MYL_NOINLINE void ImplPutArray(uint8_t *data, size_t size) {
    if constexpr (Mode == TransferMode::Polling) {
      last_status_ = HAL_UART_Transmit(huart_, data, static_cast<uint16_t>(size), timeout_);
    } else {
      // Push bytes to TX circular buffer, spinning if full
      for (size_t i = 0; i < size; ++i) {
        while (!tx_producer_.TryPut(data[i])) {}
      }
      // Critical section: check + kick must be atomic w.r.t. TxCmpltCb ISR.
      // Without this, the ISR can drain the ring and clear tx_busy_ between
      // our check and our StartTx() call, leaving tx_busy_=false after StartTx
      // returns even though hardware is still transmitting.
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      if (!tx_busy_) StartTx();
      __set_PRIMASK(primask);
    }
  }

  MYL_NOINLINE bool ImplReadable() {
    if constexpr (Mode == TransferMode::Polling) {
      return (__HAL_UART_GET_FLAG(huart_, UART_FLAG_RXNE) != 0U);
    } else {
      return rx_consumer_.Readable();
    }
  }

  MYL_NOINLINE char ImplGetC() {
    if constexpr (Mode == TransferMode::Polling) {
      uint8_t byte = 0;
      last_status_ = HAL_UART_Receive(huart_, &byte, 1, timeout_);
      return static_cast<char>(byte);
    } else {
      uint8_t b;
      while (!rx_consumer_.TryGet(b)) {}
      return static_cast<char>(b);
    }
  }

  MYL_NOINLINE bool ImplGetCTimeout(char &c, size_t timeout_ms) {
    if constexpr (Mode == TransferMode::Polling) {
      uint8_t byte = 0;
      last_status_ = HAL_UART_Receive(huart_, &byte, 1, static_cast<uint32_t>(timeout_ms));
      if (last_status_ == HAL_OK) {
        c = static_cast<char>(byte);
        return true;
      }
      return false;
    } else {
      const uint32_t start = HAL_GetTick();
      uint8_t b;
      while (!rx_consumer_.TryGet(b)) {
        if ((HAL_GetTick() - start) >= static_cast<uint32_t>(timeout_ms)) {
          return false;
        }
      }
      c = static_cast<char>(b);
      return true;
    }
  }

  MYL_NOINLINE size_t ImplGetArray(uint8_t *data, size_t max_length) {
    if constexpr (Mode == TransferMode::Polling) {
      last_status_ = HAL_UART_Receive(huart_, data, static_cast<uint16_t>(max_length), timeout_);
      return (last_status_ == HAL_OK) ? max_length : 0;
    } else {
      size_t count = 0;
      while (count < max_length && rx_consumer_.TryGet(data[count])) {
        ++count;
      }
      return count;
    }
  }

 private:
  // ---- TX staging (Interrupt / Dma modes) ------------------------------

  MYL_ISR_NOINLINE void StartTx() {
    if (tx_consumer_.Readable()) {
      tx_busy_ = true;
      tx_staging_len_ = 0;
      while (tx_staging_len_ < TxBufSize && tx_consumer_.TryGet(tx_staging_[tx_staging_len_])) {
        ++tx_staging_len_;
      }
      if constexpr (Mode == TransferMode::Dma) {
        last_status_ = HAL_UART_Transmit_DMA(huart_, tx_staging_,
                                              static_cast<uint16_t>(tx_staging_len_));
      } else {
        last_status_ = HAL_UART_Transmit_IT(huart_, tx_staging_,
                                             static_cast<uint16_t>(tx_staging_len_));
      }
    } else {
      tx_busy_ = false;
    }
  }

  // ---- Fields ----------------------------------------------------------

  UART_HandleTypeDef *huart_;
  uint32_t timeout_;
  HAL_StatusTypeDef last_status_{HAL_OK};

  // Interrupt / DMA mode RX (single-byte polling via IT)
  uint8_t rx_byte_{};
  CircularBuffer<uint8_t, RxBufSize> rx_buf_;
  BufferProducer<uint8_t> rx_producer_;
  BufferConsumer<uint8_t> rx_consumer_;

  // Interrupt / DMA mode TX (ring buffer → staging array → HAL transfer)
  CircularBuffer<uint8_t, TxBufSize> tx_buf_;
  BufferProducer<uint8_t> tx_producer_;
  BufferConsumer<uint8_t> tx_consumer_;
  uint8_t tx_staging_[TxBufSize]{};
  volatile size_t tx_staging_len_{};
  volatile bool tx_busy_{false};
};

// ---------------------------------------------------------------------------
// Convenience aliases
// ---------------------------------------------------------------------------

/// DMA TX + interrupt RX UART transport with configurable buffer sizes.
template <size_t RxBufSize = 256, size_t TxBufSize = 256>
using Stm32DmaUartTransport = Stm32UartTransport<TransferMode::Dma, RxBufSize, TxBufSize>;

/// Polling UART transport (no ISR, no ring buffers).
using Stm32PollingUartTransport = Stm32UartTransport<TransferMode::Polling, 1, 1>;

}  // namespace myl_utils
