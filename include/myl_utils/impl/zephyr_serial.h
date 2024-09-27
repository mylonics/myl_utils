#pragma once

/* Usage
 *
 * I2C Zephyr Implementation
 *
 * In the Function HAL_UART_Receive_IT, comment out __HAL_LOCK(huart);
 *
 *
 * To use this class in c file:
 *
 *
 */

#include <myl_utils/impl/usbd_init.h>
#include <myl_utils/serial.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#define HW_BUFFER_SIZE 1024

#define RX_BUFFER_SIZE 512
#define TX_BUFFER_SIZE 512

class ZephyrBasicSerialDevice : public SerialPort {
 public:
  explicit ZephyrBasicSerialDevice(const struct device *dev,
                                   bool initialize = true)
      : dev_(dev) {
    if (initialize) {
      Initialize();
    }
  };

  bool Initialize() {
    ring_buf_init(&rx_rb_, sizeof(rx_buffer_), rx_buffer_);
    ring_buf_init(&tx_rb_, sizeof(tx_buffer_), tx_buffer_);

    uart_irq_callback_user_data_set(dev_, UartIntHandler, (void *)this);
    return true;
  }

  bool Start() {
    transmitting_ = false;
    uart_irq_rx_enable(dev_);
    return true;
  };

  void PutC(char c) {
    ring_buf_put(&tx_rb_, (uint8_t *)&c, 1);
    if (!transmitting_) {
      transmitting_ = true;
      uart_irq_tx_enable(dev_);
    }
  }

  bool Readable() { return ring_buf_size_get(&rx_rb_); };

  char GetC() {
    uint8_t byte;
    ring_buf_get(&rx_rb_, &byte, 1);
    return byte;
  };

 protected:
  void NewRxPacketAvailable() {}

 private:
  const struct device *dev_;
  uint8_t rx_buffer_[RX_BUFFER_SIZE];
  uint8_t tx_buffer_[TX_BUFFER_SIZE];

  struct ring_buf rx_rb_;
  struct ring_buf tx_rb_;

  volatile bool transmitting_{false};

  static void UartIntHandler(const struct device *dev, void *user_data) {
    ZephyrBasicSerialDevice *ctx = (ZephyrBasicSerialDevice *)user_data;

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
      LOG_MODULE_DECLARE(myl_utils, CONFIG_MYL_UTILS_LOG_LEVEL);
      if (uart_irq_rx_ready(dev)) {
        int recv_len, rb_len;
        uint8_t buffer[64];
        size_t len = MIN(ring_buf_space_get(&ctx->rx_rb_), sizeof(buffer));

        if (len == 0) {
          // buffer full throw away some data
          ring_buf_get(&ctx->rx_rb_, buffer, sizeof(buffer));
          len = MIN(ring_buf_space_get(&ctx->rx_rb_), sizeof(buffer));
        }

        recv_len = uart_fifo_read(dev, buffer, len);
        if (recv_len < 0) {
          LOG_ERR("Failed to read UART FIFO");
          recv_len = 0;
        };

        rb_len = ring_buf_put(&ctx->rx_rb_, buffer, recv_len);
        if (rb_len < recv_len) {
          LOG_ERR("Drop %u bytes", recv_len - rb_len);
        }
      }

      if (uart_irq_tx_ready(dev)) {
        uint8_t buffer[64];
        int rb_len, send_len;

        rb_len = ring_buf_get(&ctx->tx_rb_, buffer, sizeof(buffer));
        if (!rb_len) {
          LOG_DBG("Ring buffer empty, disable TX IRQ");
          uart_irq_tx_disable(dev);
          ctx->transmitting_ = false;
          continue;
        }

        send_len = uart_fifo_fill(dev, buffer, rb_len);
        if (send_len < rb_len) {
          LOG_ERR("Drop %d bytes", rb_len - send_len);
        }

        LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
      }
    }
  }
};

uart_config_stop_bits ToZephyrStopBits(UartStopBits stop_bits) {
  if (stop_bits == UartStopBits::TWO) {
    return UART_CFG_STOP_BITS_2;
  }
  return UART_CFG_STOP_BITS_1;
}

uart_config_parity ToZephyrParity(UartParity parity) {
  if (parity == UartParity::NONE) {
    return UART_CFG_PARITY_NONE;
  }

  if (parity == UartParity::EVEN) {
    return UART_CFG_PARITY_EVEN;
  }

  return UART_CFG_PARITY_ODD;
}

class ZephyrSerialDevice : public ZephyrBasicSerialDevice {
 public:
  explicit ZephyrSerialDevice(const struct device *dev)
      : ZephyrBasicSerialDevice(dev), dev_(dev) {
    uart_configure(dev_, &uart_cfg_);

    Start();
  };

  void ReconfigureUart(UartBaud baudrate, UartParity parity,
                       UartStopBits stop_bits) {
    uart_irq_rx_disable(dev_);
    uart_irq_tx_disable(dev_);

    uart_cfg_.baudrate = UartBaudEnumToValue(baudrate);
    uart_cfg_.stop_bits = ToZephyrStopBits(stop_bits);
    uart_cfg_.parity = ToZephyrParity(parity);

    uart_configure(dev_, &uart_cfg_);

    Start();
  }

 private:
  const struct device *dev_;

  struct uart_config uart_cfg_ = {
      .baudrate = 9600,
      .parity = UART_CFG_PARITY_ODD,
      .stop_bits = UART_CFG_STOP_BITS_1,
      .data_bits = UART_CFG_DATA_BITS_8,
      .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
  };
};

#ifdef CONFIG_MYL_UTILS_USB_SERIAL
class ZephyrUsbSerialDevice : public ZephyrBasicSerialDevice {
 public:
  explicit ZephyrUsbSerialDevice(const struct device *dev)
      : ZephyrBasicSerialDevice(dev, false), dev_(dev) {

        };

  bool Start(bool connect_blocking = false) {
    LOG_MODULE_DECLARE(myl_utils, CONFIG_MYL_UTILS_LOG_LEVEL);
    if (!device_is_ready(dev_)) {
      LOG_ERR("CDC ACM device not ready");
      return false;
    }
    InitializeHW();

    if (connect_blocking && !connected) {
      k_sem_take(&dtr_sem, K_FOREVER);
      connected = true;
    }

    Initialize();
    uart_irq_rx_enable(dev_);
    return true;
  }

 private:
  static int InitializeHW() {
    LOG_MODULE_DECLARE(myl_utils, CONFIG_MYL_UTILS_LOG_LEVEL);
    int ret = 0;
    if (!hw_initialized) {
      connected = false;
      k_sem_init(&dtr_sem, 0, 1);
      sample_usbd = sample_usbd_init_device(usb_hw_msg_cb);
      if (sample_usbd == NULL) {
        LOG_ERR("Failed to initialize USB device");
        ret = -ENODEV;
        return ret;
      }

      if (!usbd_can_detect_vbus(sample_usbd)) {
        int err = usbd_enable(sample_usbd);
        if (err) {
          LOG_ERR("Failed to enable device support");
          ret = err;
          return ret;
        }
      }

      LOG_INF("USB device support enabled");

      k_msleep(100); /* Wait 100ms for the host to do all settings */
      hw_initialized = true;
    }
    return ret;
  }

  static void usb_hw_msg_cb(struct usbd_context *const ctx,
                            const struct usbd_msg *msg) {
    LOG_MODULE_DECLARE(myl_utils, CONFIG_MYL_UTILS_LOG_LEVEL);
    LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

    if (usbd_can_detect_vbus(ctx)) {
      if (msg->type == USBD_MSG_VBUS_READY) {
        if (usbd_enable(ctx)) {
          LOG_ERR("Failed to enable device support");
        }
      }

      if (msg->type == USBD_MSG_VBUS_REMOVED) {
        if (usbd_disable(ctx)) {
          LOG_ERR("Failed to disable device support");
        }
      }
    }

    if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
      uint32_t dtr = 0U;

      uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
      if (dtr) {
        k_sem_give(&dtr_sem);
      }
    }
  };

  const struct device *dev_;

  inline static bool connected = false;
  inline static bool hw_initialized = false;
  inline static struct usbd_context *sample_usbd;
  inline static struct k_sem dtr_sem;
};
#endif
