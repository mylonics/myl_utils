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
 * // Create device instance
 * const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
 * ZephyrI2cDevice i2c(i2c_dev);
 *
 * // Create and configure a packet
 * I2C_PACKET_HELPER(0x50, eeprom_pkt, 16);
 * eeprom_pkt_tx_data_.data[0] = 0x00;  // Register address
 * eeprom_pkt_tx_data_.length = 1;
 * eeprom_pkt_rx_data_.length = 8;
 * eeprom_pkt.type = PacketType::WriteThenRead;
 *
 * // Execute transfer
 * i2c.ProcessCommand(eeprom_pkt);
 * @endcode
 */

#include <myl_utils/peripheral.h>
#include <zephyr/drivers/i2c.h>

#ifndef CONFIG_I2C
#error Enable CONFIG_MYL_UTILS_I2C in kconfig (prj.conf)
#endif

/**
 * @brief Synchronous Zephyr I2C device implementation
 *
 * This class provides synchronous (blocking) I2C communication using Zephyr's
 * I2C driver API. For asynchronous operation, create a subclass that overrides
 * the transfer methods and inherits from AsyncI2c instead.
 */
class ZephyrI2cDevice : public I2c {
 protected:
  const struct device *dev_;
  struct i2c_msg msgs_[2];
  uint8_t msg_count{};

  /**
   * @brief Execute the prepared I2C transfer
   * @param pkt The packet containing transfer data and address
   */
  virtual void StartTransfer(I2cPacket &pkt) { i2c_transfer(dev_, msgs_, msg_count, pkt.addr); }

  void ReadWritePacket(I2cPacket &pkt) override {
    msgs_[0].buf = pkt.tx_data.data;
    msgs_[0].len = pkt.tx_data.length;
    msgs_[0].flags = I2C_MSG_WRITE;

    msgs_[1].buf = pkt.rx_data.data;
    msgs_[1].len = pkt.rx_data.length;
    msgs_[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;
    msg_count = 2;
    StartTransfer(pkt);
  }

  void WritePacket(I2cPacket &pkt) override {
    msgs_[0].buf = pkt.tx_data.data;
    msgs_[0].len = pkt.tx_data.length;
    msgs_[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
    msg_count = 1;
    StartTransfer(pkt);
  }

  void ReadPacket(I2cPacket &pkt) override {
    msgs_[0].buf = pkt.rx_data.data;
    msgs_[0].len = pkt.rx_data.length;
    msgs_[0].flags = I2C_MSG_READ | I2C_MSG_STOP;
    msg_count = 1;
    StartTransfer(pkt);
  }

  void ChipSelect(I2cPacket & /*pkt*/, bool /*enable*/) override {
    // I2C does not use chip select - addressing is handled via the packet address
  }

 public:
  /**
   * @brief Construct a new Zephyr I2C Device
   * @param i2c_dev Pointer to the Zephyr I2C device (from DEVICE_DT_GET)
   */
  explicit ZephyrI2cDevice(const struct device *i2c_dev) : I2c{}, dev_(i2c_dev) {}
};
