#pragma once

/* Usage
 * SPI Zephyr Implementation
 */

#include <myl_utils/peripheral.h>
#include <zephyr/drivers/spi.h>

#ifndef CONFIG_SPI
#pragma error "Enable CONFIG_MYL_UTILS_SPI in kconfig (prj.conf)"
#endif

class ZephyrSpiDevice : public Spi {
 protected:
  const struct device *dev_;
  const struct spi_config *config_;
  spi_buf_set tx_buf_set;
  spi_buf_set rx_buf_set;

  void start_spi_transfer(SpiPacket &pkt) {
    if (tx_buf_set.count == 0) {
      spi_transceive(dev_, config_, NULL, &rx_buf_set);
    } else if (rx_buf_set.count == 0) {
      spi_transceive(dev_, config_, &tx_buf_set, NULL);
    } else {
      spi_transceive(dev_, config_, &tx_buf_set, &rx_buf_set);
    }
  }

  void ReadWritePacketIt(SpiPacket &pkt) {
    spi_buf tx_buf{pkt.tx_data.data, pkt.tx_data.length};
    spi_buf rx_buf{pkt.rx_data.data, pkt.rx_data.length};

    tx_buf_set = {&tx_buf, 1};
    rx_buf_set = {&rx_buf, 1};

    start_spi_transfer(pkt);
  };

  void WritePacketIt(SpiPacket &pkt) {
    spi_buf tx_buf{pkt.tx_data.data, pkt.tx_data.length};

    tx_buf_set = {&tx_buf, 1};
    rx_buf_set.count = 0;

    start_spi_transfer(pkt);
  };

  void ReadPacketIt(SpiPacket &pkt) {
    spi_buf rx_buf{pkt.rx_data.data, pkt.rx_data.length};

    tx_buf_set.count = 0;
    rx_buf_set = {&rx_buf, 1};

    start_spi_transfer(pkt);
  };

  void chip_select(SpiPacket &pkt, bool enable) { pkt.chip_select(enable); }

 public:
  ZephyrSpiDevice(const struct device *spi_dev, const struct spi_config *spi_config)
      : Spi{}, dev_(spi_dev), config_(spi_config) {}
};
