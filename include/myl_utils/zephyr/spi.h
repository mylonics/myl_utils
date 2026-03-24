#pragma once

/**
 * @file spi.h
 * @brief Zephyr SPI device implementation
 *
 * Provides synchronous and asynchronous SPI device classes for Zephyr RTOS.
 * These implementations extend the Spi/AsyncSpi base classes to work with
 * the peripheral packet system.
 *
 * Usage (synchronous):
 * @code
 * // Create transport instance
 * const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
 * const struct spi_config spi_cfg = {
 *     .frequency = 1000000,
 *     .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8),
 * };
 * ZephyrSpiDevice spi(spi_dev, &spi_cfg);
 *
 * // Wrap transport with per-device config (chip select, callback)
 * ZephyrGpioOutput cs_pin(cs_spec, false);  // CS initially deasserted
 * SpiDevice<ZephyrSpiDevice> sensor(spi, cs_pin);
 *
 * // Create and configure buffers
 * Buffer<16> tx;
 * Buffer<16> rx;
 * tx.Set(0x80 | REG_ADDR);  // Read register, length = 1
 * rx.length = 4;
 *
 * // Execute transfer — chip select is applied automatically
 * auto pkt = SpiPacket::RegRead(tx, rx);
 * sensor.ProcessCommand(pkt);
 * @endcode
 *
 * Usage (asynchronous):
 * @code
 * const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
 * const struct spi_config spi_cfg = { ... };
 * ZephyrAsyncSpiDevice<> spi(spi_dev, &spi_cfg);
 *
 * ZephyrGpioOutput cs_pin(cs_spec, false);
 * SpiDevice<ZephyrAsyncSpiDevice<>> sensor(spi, cs_pin, my_callback);
 *
 * // Buffers must outlive the async transfer (use class members or static)
 * SpiPacketBundle<16> cmd;
 * cmd.tx.Set(0x80 | REG_ADDR);
 * cmd.rx.length = 4;
 * cmd.pkt.type = PacketType::WriteThenRead;
 * sensor.ProcessCommand(cmd.pkt);
 *
 * // Call PollComplete() periodically (e.g., from a Zephyr thread or timer)
 * // to check for transfer completion and advance the queue:
 * spi.PollComplete();
 * @endcode
 */

#include <myl_utils/peripheral.h>
#include <zephyr/drivers/spi.h>

#ifndef CONFIG_SPI
#error Enable CONFIG_MYL_UTILS_SPI in kconfig (prj.conf)
#endif

namespace myl_utils {

/**
 * @brief Synchronous Zephyr SPI device implementation
 *
 * This class provides synchronous (blocking) SPI communication using Zephyr's
 * SPI driver API. For asynchronous operation, create a separate class that
 * inherits from AsyncSpi<Derived, QueueSize> instead.
 */
class ZephyrSpiDevice : public Spi<ZephyrSpiDevice> {
  friend class SyncPacketSender<ZephyrSpiDevice, SpiPacket>;

 protected:
  const struct device *dev_;
  struct spi_config config_;
  int last_error_{};

  /**
   * @brief Execute an SPI transfer
   * @param tx Pointer to TX buffer set (nullptr for read-only)
   * @param rx Pointer to RX buffer set (nullptr for write-only)
   */
  bool StartTransfer(const spi_buf_set *tx, const spi_buf_set *rx) {
    last_error_ = spi_transceive(dev_, &config_, tx, rx);
    return last_error_ == 0;
  }

  MYL_NOINLINE bool ReadWritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    spi_buf rx_buf = {pkt.rx_data->data, pkt.rx_data->length};
    spi_buf_set tx_set = {&tx_buf, 1};
    spi_buf_set rx_set = {&rx_buf, 1};
    return StartTransfer(&tx_set, &rx_set);
  }

  MYL_NOINLINE bool WritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    spi_buf_set tx_set = {&tx_buf, 1};
    return StartTransfer(&tx_set, nullptr);
  }

  MYL_NOINLINE bool ReadPacket(SpiPacket &pkt) {
    spi_buf rx_buf = {pkt.rx_data->data, pkt.rx_data->length};
    spi_buf_set rx_set = {&rx_buf, 1};
    return StartTransfer(nullptr, &rx_set);
  }

  MYL_NOINLINE void ChipSelect(SpiPacket &pkt, bool enable) {
    if (enable) {
      uint16_t new_op = config_.operation;
      if (pkt.polarity == SpiPolarity::High) {
        new_op |= SPI_MODE_CPOL;
      } else {
        new_op &= ~SPI_MODE_CPOL;
      }
      if (pkt.phase == SpiPhase::Trailing) {
        new_op |= SPI_MODE_CPHA;
      } else {
        new_op &= ~SPI_MODE_CPHA;
      }
      if (new_op != config_.operation) {
        config_.operation = new_op;
      }
    }
    pkt.chip_select.Set(enable);
  }

 public:
  /**
   * @brief Construct a new Zephyr SPI Device
   * @param spi_dev Pointer to the Zephyr SPI device (from DEVICE_DT_GET)
   * @param spi_config Pointer to the SPI configuration structure (copied internally)
   */
  ZephyrSpiDevice(const struct device *spi_dev, const struct spi_config *spi_config)
      : dev_(spi_dev), config_(*spi_config) {}

  /// Get the error code from the last transfer (0 = success)
  int last_error() const { return last_error_; }

  /// Access the mutable SPI config (e.g., to change frequency at runtime)
  struct spi_config &config() { return config_; }
  const struct spi_config &config() const { return config_; }
};

#ifdef CONFIG_SPI_ASYNC
#include <zephyr/kernel.h>

/**
 * @brief Asynchronous Zephyr SPI device implementation
 *
 * Uses Zephyr's spi_transceive_signal() for non-blocking SPI transfers. The
 * underlying driver uses DMA or interrupts depending on the SoC and devicetree
 * configuration — no user action is needed for DMA memory placement on Zephyr.
 *
 * Completion is signalled via a k_poll_signal. Call PollComplete() periodically
 * (e.g., from a thread, work queue item, or timer callback) to check for
 * transfer completion, invoke the user callback, and advance the packet queue.
 *
 * @tparam QueueSize Maximum number of packets that can be queued (default: 32)
 */
template <uint8_t QueueSize = 32>
class ZephyrAsyncSpiDevice : public AsyncSpi<ZephyrAsyncSpiDevice<QueueSize>, QueueSize> {
  friend class AsyncPacketSender<ZephyrAsyncSpiDevice<QueueSize>, SpiPacket, QueueSize>;

 protected:
  const struct device *dev_;
  struct spi_config config_;
  int last_error_{};
  struct k_poll_signal signal_;
  struct k_poll_event event_;

  // Persistent buf/set storage for the in-flight transfer (must outlive the async call)
  spi_buf active_tx_buf_{};
  spi_buf active_rx_buf_{};
  spi_buf_set active_tx_set_{};
  spi_buf_set active_rx_set_{};

  bool StartAsyncTransfer(const spi_buf *tx, const spi_buf *rx) {
    k_poll_signal_reset(&signal_);

    if (tx) {
      active_tx_buf_ = *tx;
      active_tx_set_ = {&active_tx_buf_, 1};
    }
    if (rx) {
      active_rx_buf_ = *rx;
      active_rx_set_ = {&active_rx_buf_, 1};
    }

    last_error_ = spi_transceive_signal(
        dev_, &config_,
        tx ? &active_tx_set_ : nullptr,
        rx ? &active_rx_set_ : nullptr,
        &signal_);
    return last_error_ == 0;
  }

  MYL_NOINLINE bool ReadWritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    spi_buf rx_buf = {pkt.rx_data->data, pkt.rx_data->length};
    return StartAsyncTransfer(&tx_buf, &rx_buf);
  }

  MYL_NOINLINE bool WritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    return StartAsyncTransfer(&tx_buf, nullptr);
  }

  MYL_NOINLINE bool ReadPacket(SpiPacket &pkt) {
    spi_buf rx_buf = {pkt.rx_data->data, pkt.rx_data->length};
    return StartAsyncTransfer(nullptr, &rx_buf);
  }

  MYL_NOINLINE void ChipSelect(SpiPacket &pkt, bool enable) {
    if (enable) {
      uint16_t new_op = config_.operation;
      if (pkt.polarity == SpiPolarity::High) {
        new_op |= SPI_MODE_CPOL;
      } else {
        new_op &= ~SPI_MODE_CPOL;
      }
      if (pkt.phase == SpiPhase::Trailing) {
        new_op |= SPI_MODE_CPHA;
      } else {
        new_op &= ~SPI_MODE_CPHA;
      }
      if (new_op != config_.operation) {
        config_.operation = new_op;
      }
    }
    pkt.chip_select.Set(enable);
  }

 public:
  /**
   * @brief Construct a new Zephyr Async SPI Device
   * @param spi_dev Pointer to the Zephyr SPI device (from DEVICE_DT_GET)
   * @param spi_config Pointer to the SPI configuration structure (copied internally)
   */
  ZephyrAsyncSpiDevice(const struct device *spi_dev, const struct spi_config *spi_config)
      : dev_(spi_dev), config_(*spi_config) {
    k_poll_signal_init(&signal_);
    k_poll_event_init(&event_, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &signal_);
  }

  /**
   * @brief Poll for transfer completion and advance the queue
   *
   * Call this periodically from a thread, work queue, or timer. When the
   * current transfer completes, this invokes the user callback and starts
   * the next queued packet (if any).
   *
   * @return true if a transfer was completed on this call
   * @return false if no transfer completed (still in progress, or idle)
   */
  bool PollComplete() {
    unsigned int signaled = 0;
    int result = 0;
    k_poll_signal_check(&signal_, &signaled, &result);
    if (signaled) {
      last_error_ = result;
      this->TxRxCmpltCb();
      return true;
    }
    return false;
  }

  /**
   * @brief Block the calling thread until the current transfer completes
   * @param timeout Maximum time to wait (K_FOREVER for indefinite)
   * @return 0 on success, negative errno on timeout or error
   */
  int WaitComplete(k_timeout_t timeout = K_FOREVER) {
    int ret = k_poll(&event_, 1, timeout);
    if (ret == 0) {
      event_.state = K_POLL_STATE_NOT_READY;
      unsigned int signaled = 0;
      int result = 0;
      k_poll_signal_check(&signal_, &signaled, &result);
      if (signaled) {
        last_error_ = result;
        this->TxRxCmpltCb();
      }
    }
    return ret;
  }

  /// Get the error code from the last transfer (0 = success)
  int last_error() const { return last_error_; }

  /// Access the mutable SPI config (e.g., to change frequency at runtime)
  struct spi_config &config() { return config_; }
  const struct spi_config &config() const { return config_; }
};

#endif  // CONFIG_SPI_ASYNC

}  // namespace myl_utils
