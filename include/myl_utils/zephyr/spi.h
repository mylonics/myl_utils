#pragma once

/**
 * @file spi.h
 * @brief Zephyr SPI device implementation
 *
 * Provides a synchronous SPI device class for Zephyr RTOS. This implementation
 * extends the Spi base class to work with the peripheral packet system.
 *
 * Usage:
 * @code
 * // Create device instance
 * const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
 * const struct spi_config spi_cfg = {
 *     .frequency = 1000000,
 *     .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8),
 *     .cs = { .gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(my_device), cs_gpios, 0) }
 * };
 * ZephyrSpiDevice spi(spi_dev, &spi_cfg);
 *
 * // Create and configure a packet
 * SPI_PACKET_HELPER(sensor_pkt, 16);
 * sensor_pkt_tx_data_.data[0] = 0x80 | REG_ADDR;  // Read register
 * sensor_pkt_tx_data_.length = 1;
 * sensor_pkt_rx_data_.length = 4;
 * sensor_pkt.type = PacketType::WriteThenRead;
 *
 * // Execute transfer
 * spi.ProcessCommand(sensor_pkt);
 * @endcode
 */

#include <myl_utils/peripheral.h>
#include <zephyr/drivers/spi.h>

#ifndef CONFIG_SPI
#error Enable CONFIG_MYL_UTILS_SPI in kconfig (prj.conf)
#endif

/**
 * @brief Synchronous Zephyr SPI device implementation
 *
 * This class provides synchronous (blocking) SPI communication using Zephyr's
 * SPI driver API. For asynchronous operation, create a subclass that overrides
 * the transfer methods and inherits from AsyncSpi instead.
 */
class ZephyrSpiDevice : public Spi {
 protected:
  const struct device *dev_;
  const struct spi_config *config_;
  spi_buf tx_buf_{};
  spi_buf rx_buf_{};
  spi_buf_set tx_buf_set_{};
  spi_buf_set rx_buf_set_{};

  /**
   * @brief Execute the prepared SPI transfer
   */
  void StartTransfer() {
    if (tx_buf_set_.count == 0) {
      spi_transceive(dev_, config_, NULL, &rx_buf_set_);
    } else if (rx_buf_set_.count == 0) {
      spi_transceive(dev_, config_, &tx_buf_set_, NULL);
    } else {
      spi_transceive(dev_, config_, &tx_buf_set_, &rx_buf_set_);
    }
  }

  void ReadWritePacket(SpiPacket &pkt) override {
    tx_buf_ = {pkt.tx_data.data, pkt.tx_data.length};
    rx_buf_ = {pkt.rx_data.data, pkt.rx_data.length};

    tx_buf_set_ = {&tx_buf_, 1};
    rx_buf_set_ = {&rx_buf_, 1};

    StartTransfer();
  }

  void WritePacket(SpiPacket &pkt) override {
    tx_buf_ = {pkt.tx_data.data, pkt.tx_data.length};

    tx_buf_set_ = {&tx_buf_, 1};
    rx_buf_set_.count = 0;

    StartTransfer();
  }

  void ReadPacket(SpiPacket &pkt) override {
    rx_buf_ = {pkt.rx_data.data, pkt.rx_data.length};

    tx_buf_set_.count = 0;
    rx_buf_set_ = {&rx_buf_, 1};

    StartTransfer();
  }

  void ChipSelect(SpiPacket &pkt, bool enable) override {
    if (pkt.chip_select) {
      pkt.chip_select(enable);
    }
  }

 public:
  /**
   * @brief Construct a new Zephyr SPI Device
   * @param spi_dev Pointer to the Zephyr SPI device (from DEVICE_DT_GET)
   * @param spi_config Pointer to the SPI configuration structure
   */
  ZephyrSpiDevice(const struct device *spi_dev, const struct spi_config *spi_config)
      : Spi{}, dev_(spi_dev), config_(spi_config) {}
};
