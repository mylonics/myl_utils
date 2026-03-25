#pragma once

/**
 * @file i2c.h
 * @brief Zephyr I2C transport (sync-only)
 *
 * Provides a synchronous I2C transport class for Zephyr RTOS. This
 * implementation extends the SyncI2c base class to work with the peripheral
 * packet system.
 *
 * Usage:
 * @code
 * const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
 * ZephyrI2cTransport i2c(i2c_dev);
 *
 * I2cDevice<ZephyrI2cTransport> eeprom(i2c, 0x50);
 *
 * Buffer<16> tx, rx;
 * tx.Set(0x00);
 * rx.length = 8;
 * auto pkt = I2cPacket::RegRead(tx, rx);
 * eeprom.SendSync(pkt);
 * @endcode
 */

#include <myl_utils/peripheral.h>
#include <zephyr/drivers/i2c.h>

#ifndef CONFIG_I2C
#error Enable CONFIG_MYL_UTILS_I2C in kconfig (prj.conf)
#endif

namespace myl_utils {

/**
 * @brief Synchronous Zephyr I2C transport
 *
 * Provides blocking I2C communication using Zephyr's I2C driver API.
 */
class ZephyrI2cTransport : public SyncI2c<ZephyrI2cTransport> {
  friend class SyncPacketSender<ZephyrI2cTransport, I2cPacket>;

 protected:
  const struct device *dev_;
  int last_error_{};

  bool StartTransfer(struct i2c_msg *msgs, uint8_t count, I2cPacket &pkt) {
    last_error_ = i2c_transfer(dev_, msgs, count, pkt.addr);
    return last_error_ == 0;
  }

  MYL_NOINLINE bool SyncReadWritePacket(I2cPacket &pkt) {
    struct i2c_msg msgs[2];
    msgs[0].buf = pkt.tx_data->data;
    msgs[0].len = pkt.tx_data->length;
    msgs[0].flags = I2C_MSG_WRITE;

    msgs[1].buf = pkt.rx_data->data;
    msgs[1].len = pkt.rx_data->length;
    msgs[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;
    return StartTransfer(msgs, 2, pkt);
  }

  MYL_NOINLINE bool SyncWritePacket(I2cPacket &pkt) {
    struct i2c_msg msg;
    msg.buf = pkt.tx_data->data;
    msg.len = pkt.tx_data->length;
    msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;
    return StartTransfer(&msg, 1, pkt);
  }

  MYL_NOINLINE bool SyncReadPacket(I2cPacket &pkt) {
    struct i2c_msg msg;
    msg.buf = pkt.rx_data->data;
    msg.len = pkt.rx_data->length;
    msg.flags = I2C_MSG_READ | I2C_MSG_STOP;
    return StartTransfer(&msg, 1, pkt);
  }

  MYL_NOINLINE void ChipSelect(I2cPacket & /*pkt*/, bool /*enable*/) {
    // I2C does not use chip select - addressing is handled via the packet address
  }

 public:
  explicit ZephyrI2cTransport(const struct device *i2c_dev) : dev_(i2c_dev) {}

  /// Get the error code from the last transfer (0 = success)
  int last_error() const { return last_error_; }
};

}  // namespace myl_utils
