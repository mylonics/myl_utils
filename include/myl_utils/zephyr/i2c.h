#pragma once

/**
 * @file i2c.h
 * @brief Zephyr I2C device implementation
 *
 * Provides a synchronous I2C device class for Zephyr RTOS. This implementation
 * extends the I2c base class to work with the peripheral packet system.
 *
 * Usage:
 * @code
 * // Create transport instance
 * const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
 * ZephyrI2cDevice i2c(i2c_dev);
 *
 * // Wrap transport with per-device config (address, callback)
 * I2cDevice<ZephyrI2cDevice> eeprom(i2c, 0x50);
 *
 * // Create and configure buffers
 * Buffer<16> tx;
 * Buffer<16> rx;
 * tx.Set(0x00);  // Register address, length = 1
 * rx.length = 8;
 *
 * // Execute transfer — address is applied automatically
 * auto pkt = I2cPacket::RegRead(tx, rx);
 * eeprom.ProcessCommand(pkt);
 * @endcode
 */

#include <myl_utils/peripheral.h>
#include <zephyr/drivers/i2c.h>

#ifndef CONFIG_I2C
#error Enable CONFIG_MYL_UTILS_I2C in kconfig (prj.conf)
#endif

namespace myl_utils {

/**
 * @brief Synchronous Zephyr I2C device implementation
 *
 * This class provides synchronous (blocking) I2C communication using Zephyr's
 * I2C driver API. For asynchronous operation, create a separate class that
 * inherits from AsyncI2c<Derived, QueueSize> instead.
 */
class ZephyrI2cDevice : public I2c<ZephyrI2cDevice> {
  friend class SyncPacketSender<ZephyrI2cDevice, I2cPacket>;

 protected:
  const struct device *dev_;
  int last_error_{};

  /**
   * @brief Execute the prepared I2C transfer
   * @param msgs Pointer to I2C message array
   * @param count Number of messages
   * @param pkt The packet containing the device address
   */
  bool StartTransfer(struct i2c_msg *msgs, uint8_t count, I2cPacket &pkt) {
    last_error_ = i2c_transfer(dev_, msgs, count, pkt.addr);
    return last_error_ == 0;
  }

  /**
   * @brief Write-then-read using I2C RESTART condition
   *
   * @note On I2C, ReadWritePacket and WriteThenRead are functionally identical
   * (both use a RESTART between write and read phases). There is no true
   * simultaneous full-duplex on I2C.
   */
  MYL_NOINLINE bool ReadWritePacket(I2cPacket &pkt) {
    struct i2c_msg msgs[2];
    msgs[0].buf = pkt.tx_data->data;
    msgs[0].len = pkt.tx_data->length;
    msgs[0].flags = I2C_MSG_WRITE;

    msgs[1].buf = pkt.rx_data->data;
    msgs[1].len = pkt.rx_data->length;
    msgs[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;
    return StartTransfer(msgs, 2, pkt);
  }

  MYL_NOINLINE bool WritePacket(I2cPacket &pkt) {
    struct i2c_msg msg;
    msg.buf = pkt.tx_data->data;
    msg.len = pkt.tx_data->length;
    msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;
    return StartTransfer(&msg, 1, pkt);
  }

  MYL_NOINLINE bool ReadPacket(I2cPacket &pkt) {
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
  /**
   * @brief Construct a new Zephyr I2C Device
   * @param i2c_dev Pointer to the Zephyr I2C device (from DEVICE_DT_GET)
   */
  explicit ZephyrI2cDevice(const struct device *i2c_dev) : dev_(i2c_dev) {}

  /// Get the error code from the last transfer (0 = success)
  int last_error() const { return last_error_; }
};

}  // namespace myl_utils
