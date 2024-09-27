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

#include <utils/peripheral.h>
#include <zephyr/drivers/i2c.h>

class ZephyrI2cDevice : public I2c {
 protected:
  const struct device *dev_;
  struct i2c_msg msgs_[2];
  uint8_t msg_count{};

  virtual void start_i2c_transfer(I2cPacket &pkt) { i2c_transfer(dev_, msgs_, msg_count, pkt.addr); }

  void ReadWritePacketIt(I2cPacket &pkt) {
    msgs_[0].buf = pkt.tx_data.data;
    msgs_[0].len = pkt.tx_data.length;
    msgs_[0].flags = I2C_MSG_WRITE;

    msgs_[1].buf = pkt.rx_data.data;
    msgs_[1].len = pkt.rx_data.length;
    msgs_[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;
    msg_count = 2;
    start_i2c_transfer(pkt);
  };

  void WritePacketIt(I2cPacket &pkt) {
    msgs_[0].buf = pkt.tx_data.data;
    msgs_[0].len = pkt.tx_data.length;
    msgs_[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
    msg_count = 1;
    start_i2c_transfer(pkt);
  };

  void ReadPacketIt(I2cPacket &pkt) {
    msgs_[0].buf = pkt.rx_data.data;
    msgs_[0].len = pkt.rx_data.length;
    msgs_[0].flags = I2C_MSG_READ | I2C_MSG_STOP;
    msg_count = 1;
    start_i2c_transfer(pkt);
  };

  void chip_select(I2cPacket &pkt, bool enable) {
    (void)pkt;
    (void)enable;
  }

 public:
  ZephyrI2cDevice(const struct device *i2c_dev) : I2c{}, dev_(i2c_dev) {}
};
