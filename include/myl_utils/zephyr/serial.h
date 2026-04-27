#pragma once

/**
 * @file zephyr/serial.h
 * @brief Zephyr UART transports using UartBase<Derived> CRTP (zero virtual overhead)
 *
 * Three concrete transports are provided:
 *
 *  ZephyrUartTransport        — interrupt-driven UART using Zephyr ring_buf + uart_irq_*
 *                               Requires CONFIG_MYL_UTILS_SERIAL
 *
 *  ZephyrAsyncUartTransport   — DMA/async UART using Zephyr's UART Async API
 *                               (uart_callback_set / uart_rx_enable / uart_tx)
 *                               Requires CONFIG_MYL_UTILS_UART_ASYNC
 *
 *  ZephyrUsbSerialDevice      — CDC ACM USB-serial adapter
 *                               Requires CONFIG_MYL_UTILS_USB_SERIAL
 *
 * Usage (interrupt-driven):
 * @code
 * ZephyrUartTransport uart(DEVICE_DT_GET(DT_CHOSEN(zephyr_console)));
 * uart.Start();
 * uart.PutArray(reinterpret_cast<uint8_t*>("Hello\r\n"), 7);
 * char c; uart.GetC(c, 100); // wait up to 100 ms
 * @endcode
 *
 * Usage (async/DMA):
 * @code
 * ZephyrAsyncUartTransport<> uart(DEVICE_DT_GET(DT_NODELABEL(uart1)));
 * uart.Start();
 * uart.PutArray(reinterpret_cast<uint8_t*>("Hi\r\n"), 4);
 * @endcode
 */

#include <myl_utils/noncopyable.h>
#include <myl_utils/serial.h>

#ifndef CONFIG_MYL_UTILS_SERIAL
#error "Enable CONFIG_MYL_UTILS_SERIAL in Kconfig (prj.conf)"
#endif

#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

namespace myl_utils {

// ---------------------------------------------------------------------------
// Zephyr config conversion helpers (shared across transports)
// ---------------------------------------------------------------------------

inline uart_config_stop_bits ToZephyrStopBits(UartStopBits sb) {
  return (sb == UartStopBits::TWO) ? UART_CFG_STOP_BITS_2 : UART_CFG_STOP_BITS_1;
}

inline uart_config_parity ToZephyrParity(UartParity p) {
  if (p == UartParity::EVEN) return UART_CFG_PARITY_EVEN;
  if (p == UartParity::ODD)  return UART_CFG_PARITY_ODD;
  return UART_CFG_PARITY_NONE;
}

inline uart_config_data_bits ToZephyrDataBits(UartDataBits db) {
  return static_cast<uart_config_data_bits>(db);
}

// ---------------------------------------------------------------------------
// Internal helper: shared ring-buffer state used by interrupt-driven transports
// ---------------------------------------------------------------------------

static constexpr size_t kHwChunkSize   = 512;
static constexpr size_t kRxRingBufSize = 2048;
static constexpr size_t kTxRingBufSize = 2048;

/**
 * @brief Shared ring-buffer state for interrupt-driven Zephyr UART transports.
 *
 * Not a public type — used as a private base by ZephyrUartTransport and
 * ZephyrUsbSerialDevice to avoid code duplication while keeping CRTP clean.
 */
class ZephyrUartBuffers : NonCopyable<ZephyrUartBuffers> {
 protected:
  ZephyrUartBuffers() {
    k_sem_init(&new_rx_data_, 0, 1);
    ring_buf_init(&rx_rb_, sizeof(rx_buf_), rx_buf_);
    ring_buf_init(&tx_rb_, sizeof(tx_buf_), tx_buf_);
  }

  void DoPutC(char c) {
    uint8_t b = static_cast<uint8_t>(c);
    // Spin until the TX ring has space — prevents silent byte drops when the
    // ring is full. The ISR drains the ring so space will open up.
    while (ring_buf_put(&tx_rb_, &b, 1) == 0) { StartTxIfIdle(); }
    StartTxIfIdle();
  }

  void DoPutArray(const uint8_t *data, size_t size) {
    // Write in chunks, spinning when the ring is full, to prevent silent drops.
    size_t written = 0;
    while (written < size) {
      uint32_t n = ring_buf_put(&tx_rb_, data + written, size - written);
      written += n;
      if (n > 0) StartTxIfIdle();
    }
  }

  bool DoReadable() const { return ring_buf_size_get(&rx_rb_) > 0; }

  char DoGetC() {
    uint8_t byte;
    ring_buf_get(&rx_rb_, &byte, 1);
    return static_cast<char>(byte);
  }

  bool DoGetCTimeout(char &c, size_t timeout_ms) {
    if (DoReadable() || (k_sem_take(&new_rx_data_, K_MSEC(timeout_ms)) == 0)) {
      uint8_t byte;
      ring_buf_get(&rx_rb_, &byte, 1);
      c = static_cast<char>(byte);
      return true;
    }
    return false;
  }

  size_t DoGetArray(uint8_t *data, size_t max_length) {
    size_t avail = ring_buf_size_get(&rx_rb_);
    size_t count = (avail < max_length) ? avail : max_length;
    ring_buf_get(&rx_rb_, data, count);
    return count;
  }

  void StartTxIfIdle() {
    if (!transmitting_) {
      transmitting_ = true;
      uart_irq_tx_enable(dev_);
    }
  }

  /// Shared IRQ handler for both ZephyrUartTransport and ZephyrUsbSerialDevice.
  static void IrqHandler(const struct device *dev, void *user_data) {
    auto *ctx = static_cast<ZephyrUartBuffers *>(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
      if (uart_irq_rx_ready(dev)) {
        uint8_t *data;
        size_t space = ring_buf_put_claim(&ctx->rx_rb_, &data, kHwChunkSize);
        if (space > 0) {
          size_t rx_len = uart_fifo_read(dev, data, space);
          ring_buf_put_finish(&ctx->rx_rb_, rx_len);
        } else {
          // Overflow — drain HW FIFO to avoid infinite IRQ loop
          uint8_t discard[16];
          uart_fifo_read(dev, discard, sizeof(discard));
        }
        k_sem_give(&ctx->new_rx_data_);
      }

      if (uart_irq_tx_ready(dev)) {
        uint8_t *data;
        size_t tx_len = ring_buf_get_claim(&ctx->tx_rb_, &data, kHwChunkSize);
        if (tx_len > 0) {
          size_t sent = uart_fifo_fill(dev, data, tx_len);
          ring_buf_get_finish(&ctx->tx_rb_, sent);
        } else {
          ctx->transmitting_ = false;
          uart_irq_tx_disable(dev);
        }
      }
    }
  }

  const struct device *dev_{nullptr};
  uint8_t rx_buf_[kRxRingBufSize];
  uint8_t tx_buf_[kTxRingBufSize];
  struct ring_buf rx_rb_;
  struct ring_buf tx_rb_;
  struct k_sem new_rx_data_;
  volatile bool transmitting_{false};
};

// ---------------------------------------------------------------------------
// ZephyrUartTransport — interrupt-driven hardware UART
// ---------------------------------------------------------------------------

/**
 * @brief Interrupt-driven Zephyr UART transport.
 *
 * Requires CONFIG_MYL_UTILS_SERIAL (selects UART_INTERRUPT_DRIVEN, RING_BUFFER).
 * Supports runtime reconfiguration via uart_configure().
 */
class ZephyrUartTransport
    : public UartBase<ZephyrUartTransport>,
      private ZephyrUartBuffers {
  friend class UartBase<ZephyrUartTransport>;

 public:
  /**
   * @param dev        Zephyr UART device (e.g. DEVICE_DT_GET(DT_NODELABEL(uart0)))
   * @param auto_start If true, call Start() immediately from the constructor.
   */
  explicit ZephyrUartTransport(const struct device *dev, bool auto_start = false) {
    dev_ = dev;
    uart_irq_callback_user_data_set(dev_, IrqHandler, static_cast<ZephyrUartBuffers *>(this));
    if (auto_start) ImplStart();
  }

 protected:
  bool ImplStart() {
    transmitting_ = false;
    uart_irq_rx_enable(dev_);
    return true;
  }

  void ImplReconfigureUart(UartBaud baud, UartParity parity, UartStopBits stop_bits) {
    uart_irq_rx_disable(dev_);
    uart_irq_tx_disable(dev_);

    struct uart_config cfg = {
        .baudrate  = static_cast<uint32_t>(UartBaudEnumToValue(baud)),
        .parity    = ToZephyrParity(parity),
        .stop_bits = ToZephyrStopBits(stop_bits),
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    uart_configure(dev_, &cfg);
    ImplStart();
  }

  void   ImplPutC(char c)                           { DoPutC(c); }
  void   ImplPutArray(uint8_t *data, size_t size)   { DoPutArray(data, size); }
  bool   ImplReadable()                             { return DoReadable(); }
  char   ImplGetC()                                 { return DoGetC(); }
  bool   ImplGetCTimeout(char &c, size_t ms)        { return DoGetCTimeout(c, ms); }
  size_t ImplGetArray(uint8_t *data, size_t max)    { return DoGetArray(data, max); }
};

// ---------------------------------------------------------------------------
// ZephyrAsyncUartTransport — UART Async API (DMA-capable)
// ---------------------------------------------------------------------------

#ifdef CONFIG_MYL_UTILS_UART_ASYNC

/**
 * @brief Zephyr UART Async API transport (DMA-capable).
 *
 * Requires CONFIG_MYL_UTILS_UART_ASYNC (selects UART_ASYNC_API).
 *
 * Uses double-buffered DMA RX (two buffers of RxBufSize alternated on
 * UART_RX_BUF_REQUEST) and a consumer ring buffer of ConsumerBufSize.
 * TX sends directly from the caller's buffer via uart_tx() (zero-copy);
 * PutArray() blocks until the previous TX completes before starting a new one.
 *
 * @tparam RxBufSize      Size of each of the two DMA RX staging buffers (bytes)
 * @tparam ConsumerBufSize Size of the ring buffer exposed to the application (power of 2)
 */
template <size_t RxBufSize = 256, size_t ConsumerBufSize = 1024>
class ZephyrAsyncUartTransport
    : public UartBase<ZephyrAsyncUartTransport<RxBufSize, ConsumerBufSize>>,
      NonCopyable<ZephyrAsyncUartTransport<RxBufSize, ConsumerBufSize>> {
  friend class UartBase<ZephyrAsyncUartTransport<RxBufSize, ConsumerBufSize>>;

  static_assert((ConsumerBufSize & (ConsumerBufSize - 1)) == 0,
                "ConsumerBufSize must be a power of 2");

 public:
  explicit ZephyrAsyncUartTransport(const struct device *dev) : dev_(dev) {
    ring_buf_init(&rx_rb_, sizeof(consumer_buf_), consumer_buf_);
    k_sem_init(&rx_sem_, 0, 1);
    k_sem_init(&tx_done_sem_, 1, 1);  // starts available — no TX in progress
    uart_callback_set(dev_, AsyncCallback, this);
  }

  /// Returns true if the previous PutArray() TX has finished.
  bool TxDone() const { return k_sem_count_get(&tx_done_sem_) > 0; }

  /// Returns the number of RX bytes lost due to the consumer ring buffer being full.
  uint32_t DroppedRxBytes() const { return dropped_rx_bytes_; }

 protected:
  bool ImplStart() {
    active_rx_ = 0;
    return uart_rx_enable(dev_, rx_dma_buf_[0], RxBufSize, SYS_FOREVER_US) == 0;
  }

  void ImplReconfigureUart(UartBaud baud, UartParity parity, UartStopBits stop_bits) {
    uart_rx_disable(dev_);
    struct uart_config cfg = {
        .baudrate  = static_cast<uint32_t>(UartBaudEnumToValue(baud)),
        .parity    = ToZephyrParity(parity),
        .stop_bits = ToZephyrStopBits(stop_bits),
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    uart_configure(dev_, &cfg);
    ImplStart();
  }

  void ImplPutC(char c) { ImplPutArray(reinterpret_cast<uint8_t *>(&c), 1); }

  void ImplPutArray(uint8_t *data, size_t size) {
    // Block until any in-progress TX completes, then start the new transfer.
    k_sem_take(&tx_done_sem_, K_FOREVER);
    uart_tx(dev_, data, size, SYS_FOREVER_US);
    // tx_done_sem_ is given back in UART_TX_DONE callback.
  }

  bool ImplReadable() { return ring_buf_size_get(&rx_rb_) > 0; }

  char ImplGetC() {
    while (!ImplReadable()) { k_sem_take(&rx_sem_, K_FOREVER); }
    uint8_t byte;
    ring_buf_get(&rx_rb_, &byte, 1);
    return static_cast<char>(byte);
  }

  bool ImplGetCTimeout(char &c, size_t timeout_ms) {
    if (ImplReadable() || (k_sem_take(&rx_sem_, K_MSEC(timeout_ms)) == 0)) {
      uint8_t byte;
      ring_buf_get(&rx_rb_, &byte, 1);
      c = static_cast<char>(byte);
      return true;
    }
    return false;
  }

  size_t ImplGetArray(uint8_t *data, size_t max_length) {
    size_t avail = ring_buf_size_get(&rx_rb_);
    size_t count = (avail < max_length) ? avail : max_length;
    ring_buf_get(&rx_rb_, data, count);
    return count;
  }

 private:
  static void AsyncCallback(const struct device *dev, struct uart_event *evt, void *user_data) {
    auto *self = static_cast<ZephyrAsyncUartTransport *>(user_data);
    switch (evt->type) {
      case UART_TX_DONE:
        k_sem_give(&self->tx_done_sem_);
        break;

      case UART_RX_RDY: {
        uint32_t written = ring_buf_put(&self->rx_rb_,
                                        evt->data.rx.buf + evt->data.rx.offset,
                                        evt->data.rx.len);
        self->dropped_rx_bytes_ += (evt->data.rx.len - written);
        k_sem_give(&self->rx_sem_);
        break;
      }

      case UART_RX_BUF_REQUEST: {
        // Provide the alternate DMA staging buffer
        uint8_t next = self->active_rx_ ^ 1u;
        uart_rx_buf_rsp(dev, self->rx_dma_buf_[next], RxBufSize);
        self->active_rx_ = next;
        break;
      }

      case UART_RX_DISABLED:
        // Stopped during reconfiguration — nothing to do.
        break;

      default:
        break;
    }
  }

  const struct device *dev_;
  uint8_t rx_dma_buf_[2][RxBufSize]{};
  volatile uint8_t active_rx_{0};  // written in async callback (ISR context)

  uint8_t consumer_buf_[ConsumerBufSize]{};
  struct ring_buf rx_rb_;
  struct k_sem rx_sem_;
  struct k_sem tx_done_sem_;
  uint32_t dropped_rx_bytes_{0};
};

#endif  // CONFIG_MYL_UTILS_UART_ASYNC

// ---------------------------------------------------------------------------
// ZephyrUsbSerialDevice — CDC ACM USB serial (requires USB_DEVICE_STACK_NEXT)
// ---------------------------------------------------------------------------

#ifdef CONFIG_MYL_UTILS_USB_SERIAL

#include <myl_utils/zephyr/usbd_init.h>
#include <myl_utils/log.h>

/**
 * @brief USB CDC ACM serial transport.
 *
 * Requires CONFIG_MYL_UTILS_USB_SERIAL (selects USB_DEVICE_STACK_NEXT + MYL_UTILS_SERIAL).
 *
 * Usage:
 * @code
 * ZephyrUsbSerialDevice usb(DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0)));
 * usb.Start(true);  // blocks until host opens port (DTR)
 * usb.PutArray(reinterpret_cast<uint8_t*>("Hello USB\r\n"), 11);
 * @endcode
 */
class ZephyrUsbSerialDevice
    : public UartBase<ZephyrUsbSerialDevice>,
      private ZephyrUartBuffers {
  friend class UartBase<ZephyrUsbSerialDevice>;

 public:
  explicit ZephyrUsbSerialDevice(const struct device *dev) {
    dev_ = dev;
  }

  /**
   * @param connect_blocking If true, block until the host opens the port (DTR asserted).
   */
  bool Start(bool connect_blocking = false) {
    DECLARE_MYL_UTILS_LOG();
    if (!device_is_ready(dev_)) {
      LOG_ERR("CDC ACM device not ready");
      return false;
    }
    InitializeHW();

    if (connect_blocking && !connected_) {
      k_sem_take(&dtr_sem_, K_FOREVER);
      connected_ = true;
    }

    uart_irq_callback_user_data_set(dev_, IrqHandler, static_cast<ZephyrUartBuffers *>(this));
    uart_irq_rx_enable(dev_);
    return true;
  }

 protected:
  bool ImplStart() { return Start(false); }

  void ImplReconfigureUart(UartBaud /*baud*/, UartParity /*parity*/, UartStopBits /*stop_bits*/) {
    // USB CDC ACM baud rate is host-controlled — reconfiguration is a no-op.
  }

  void   ImplPutC(char c)                           { DoPutC(c); }
  void   ImplPutArray(uint8_t *data, size_t size)   { DoPutArray(data, size); }
  bool   ImplReadable()                             { return DoReadable(); }
  char   ImplGetC()                                 { return DoGetC(); }
  bool   ImplGetCTimeout(char &c, size_t ms)        { return DoGetCTimeout(c, ms); }
  size_t ImplGetArray(uint8_t *data, size_t max)    { return DoGetArray(data, max); }

 private:
  static int InitializeHW() {
    DECLARE_MYL_UTILS_LOG();
    if (hw_initialized_) return 0;

    connected_ = false;
    k_sem_init(&dtr_sem_, 0, 1);
    sample_usbd_ = sample_usbd_init_device(UsbHwMsgCb);
    if (!sample_usbd_) {
      LOG_ERR("Failed to initialize USB device");
      return -ENODEV;
    }

    if (!usbd_can_detect_vbus(sample_usbd_)) {
      int err = usbd_enable(sample_usbd_);
      if (err) {
        LOG_ERR("Failed to enable device support");
        return err;
      }
    }

    LOG_INF("USB device support enabled");
    k_msleep(100);
    hw_initialized_ = true;
    return 0;
  }

  static void UsbHwMsgCb(struct usbd_context *const ctx, const struct usbd_msg *msg) {
    DECLARE_MYL_UTILS_LOG();
    LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

    if (usbd_can_detect_vbus(ctx)) {
      if (msg->type == USBD_MSG_VBUS_READY) {
        if (usbd_enable(ctx)) LOG_ERR("Failed to enable device support");
      } else if (msg->type == USBD_MSG_VBUS_REMOVED) {
        if (usbd_disable(ctx)) LOG_ERR("Failed to disable device support");
      }
    }

    if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
      uint32_t dtr = 0U;
      uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
      if (dtr) k_sem_give(&dtr_sem_);
    }
  }

  inline static bool connected_{false};
  inline static bool hw_initialized_{false};
  inline static struct usbd_context *sample_usbd_{nullptr};
  inline static struct k_sem dtr_sem_;
};

#endif  // CONFIG_MYL_UTILS_USB_SERIAL

}  // namespace myl_utils

