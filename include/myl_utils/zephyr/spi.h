#pragma once

/**
 * @file spi.h
 * @brief Zephyr SPI unified transport (sync + optionally async)
 *
 * Provides SPI transport classes for Zephyr RTOS. ZephyrSpiTransport is
 * sync-only (no queue overhead). When CONFIG_SPI_ASYNC is enabled,
 * ZephyrAsyncSpiTransport provides both sync and async through a single
 * object by inheriting from both SyncSpi and AsyncSpi CRTP bases.
 *
 * Usage (sync-only):
 * @code
 * const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
 * const struct spi_config spi_cfg = { .frequency = 1000000, .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) };
 * ZephyrSpiTransport spi(spi_dev, &spi_cfg);
 *
 * ZephyrGpioOutput cs_pin(cs_spec, false);
 * SpiDevice<ZephyrSpiTransport> sensor(spi, ChipSelectPin(cs_pin));
 *
 * auto pkt = SpiPacket::RegRead(tx, rx);
 * sensor.SendSync(pkt);
 * @endcode
 *
 * Usage (async, requires CONFIG_SPI_ASYNC):
 * @code
 * const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
 * const struct spi_config spi_cfg = { ... };
 * ZephyrAsyncSpiTransport<> spi(spi_dev, &spi_cfg);
 *
 * SpiDevice<ZephyrAsyncSpiTransport<>> sensor(spi, ChipSelectPin(cs_pin), my_callback);
 *
 * // Sync during init
 * sensor.SendSync(pkt);
 *
 * // Async during runtime
 * sensor.SendAsync(cmd.pkt);
 *
 * // Poll for completion periodically:
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
 * @brief Synchronous-only Zephyr SPI transport
 *
 * Provides blocking SPI communication using Zephyr's spi_transceive(). For
 * async+sync support, use ZephyrAsyncSpiTransport (requires CONFIG_SPI_ASYNC).
 */
class ZephyrSpiTransport : public SyncSpi<ZephyrSpiTransport> {
  friend class SyncPacketSender<ZephyrSpiTransport, SpiPacket>;

 protected:
  const struct device *dev_;
  struct spi_config config_;
  const uint32_t default_freq_;
  int last_error_{};

  bool StartTransfer(const spi_buf_set *tx, const spi_buf_set *rx) {
    last_error_ = spi_transceive(dev_, &config_, tx, rx);
    return last_error_ == 0;
  }

  MYL_NOINLINE bool SyncReadWritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    spi_buf rx_buf = {pkt.rx_data->data, pkt.rx_data->length};
    spi_buf_set tx_set = {&tx_buf, 1};
    spi_buf_set rx_set = {&rx_buf, 1};
    return StartTransfer(&tx_set, &rx_set);
  }

  MYL_NOINLINE bool SyncWritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    spi_buf_set tx_set = {&tx_buf, 1};
    return StartTransfer(&tx_set, nullptr);
  }

  MYL_NOINLINE bool SyncReadPacket(SpiPacket &pkt) {
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

      // Frequency override (only slow down, never exceed default)
      uint32_t target_freq = default_freq_;
      if (pkt.max_freq_hz > 0 && pkt.max_freq_hz < default_freq_) {
        target_freq = pkt.max_freq_hz;
      }
      config_.frequency = target_freq;
    }
    pkt.chip_select.Set(enable);
  }

 public:
  ZephyrSpiTransport(const struct device *spi_dev, const struct spi_config *spi_config)
      : dev_(spi_dev), config_(*spi_config), default_freq_(spi_config->frequency) {}

  /// Get the error code from the last transfer (0 = success)
  int last_error() const { return last_error_; }

  /// Access the mutable SPI config (e.g., to change frequency at runtime)
  struct spi_config &config() { return config_; }
  const struct spi_config &config() const { return config_; }
};

#ifdef CONFIG_SPI_ASYNC
#include <zephyr/kernel.h>

/**
 * @brief Unified Zephyr SPI transport (sync + async)
 *
 * Uses Zephyr's spi_transceive() for blocking (sync) and spi_transceive_signal()
 * for non-blocking (async) transfers. Both paths are available through a single
 * object.
 *
 * For async: call PollComplete() periodically (from a thread, work queue, or timer)
 * to check for completion, invoke the callback, and advance the packet queue.
 * Alternatively, call WaitComplete() to block until the current transfer finishes.
 *
 * @tparam QueueSize Maximum number of async packets that can be queued (default: 32)
 */
template <uint8_t QueueSize = 32>
class ZephyrAsyncSpiTransport
    : public SyncSpi<ZephyrAsyncSpiTransport<QueueSize>>
    , public AsyncSpi<ZephyrAsyncSpiTransport<QueueSize>, QueueSize> {
  friend class SyncPacketSender<ZephyrAsyncSpiTransport<QueueSize>, SpiPacket>;
  friend class AsyncPacketSender<ZephyrAsyncSpiTransport<QueueSize>, SpiPacket, QueueSize>;

 protected:
  const struct device *dev_;
  struct spi_config config_;
  const uint32_t default_freq_;
  int last_error_{};
  struct k_poll_signal signal_;
  struct k_poll_event event_;

  // Persistent buf/set storage for the in-flight async transfer (must outlive the async call)
  spi_buf active_tx_buf_{};
  spi_buf active_rx_buf_{};
  spi_buf_set active_tx_set_{};
  spi_buf_set active_rx_set_{};

  // ---- Shared chip select (same for sync and async paths) ----------------

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

      // Frequency override (only slow down, never exceed default)
      uint32_t target_freq = default_freq_;
      if (pkt.max_freq_hz > 0 && pkt.max_freq_hz < default_freq_) {
        target_freq = pkt.max_freq_hz;
      }
      config_.frequency = target_freq;
    }
    pkt.chip_select.Set(enable);
  }

  // ---- Synchronous (blocking) implementations ---------------------------

  bool SyncStartTransfer(const spi_buf_set *tx, const spi_buf_set *rx) {
    last_error_ = spi_transceive(dev_, &config_, tx, rx);
    return last_error_ == 0;
  }

  MYL_NOINLINE bool SyncReadWritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    spi_buf rx_buf = {pkt.rx_data->data, pkt.rx_data->length};
    spi_buf_set tx_set = {&tx_buf, 1};
    spi_buf_set rx_set = {&rx_buf, 1};
    return SyncStartTransfer(&tx_set, &rx_set);
  }

  MYL_NOINLINE bool SyncWritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    spi_buf_set tx_set = {&tx_buf, 1};
    return SyncStartTransfer(&tx_set, nullptr);
  }

  MYL_NOINLINE bool SyncReadPacket(SpiPacket &pkt) {
    spi_buf rx_buf = {pkt.rx_data->data, pkt.rx_data->length};
    spi_buf_set rx_set = {&rx_buf, 1};
    return SyncStartTransfer(nullptr, &rx_set);
  }

  // ---- Asynchronous implementations -------------------------------------

  bool AsyncStartTransfer(const spi_buf *tx, const spi_buf *rx) {
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

  MYL_NOINLINE bool AsyncReadWritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    spi_buf rx_buf = {pkt.rx_data->data, pkt.rx_data->length};
    return AsyncStartTransfer(&tx_buf, &rx_buf);
  }

  MYL_NOINLINE bool AsyncWritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    return AsyncStartTransfer(&tx_buf, nullptr);
  }

  MYL_NOINLINE bool AsyncReadPacket(SpiPacket &pkt) {
    spi_buf rx_buf = {pkt.rx_data->data, pkt.rx_data->length};
    return AsyncStartTransfer(nullptr, &rx_buf);
  }

 public:
  ZephyrAsyncSpiTransport(const struct device *spi_dev, const struct spi_config *spi_config)
      : dev_(spi_dev), config_(*spi_config), default_freq_(spi_config->frequency) {
    k_poll_signal_init(&signal_);
    k_poll_event_init(&event_, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &signal_);
  }

  /**
   * @brief Poll for async transfer completion and advance the queue
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
   * @brief Block the calling thread until the current async transfer completes
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
