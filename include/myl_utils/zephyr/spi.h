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
 * sensor.WriteThenRead(tx, rx);
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

  bool ReadWritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    spi_buf rx_buf = {pkt.rx_data->data, pkt.rx_data->length};
    spi_buf_set tx_set = {&tx_buf, 1};
    spi_buf_set rx_set = {&rx_buf, 1};
    return StartTransfer(&tx_set, &rx_set);
  }

  bool WritePacket(SpiPacket &pkt) {
    spi_buf tx_buf = {pkt.tx_data->data, pkt.tx_data->length};
    spi_buf_set tx_set = {&tx_buf, 1};
    return StartTransfer(&tx_set, nullptr);
  }

  bool ReadPacket(SpiPacket &pkt) {
    spi_buf rx_buf = {pkt.rx_data->data, pkt.rx_data->length};
    spi_buf_set rx_set = {&rx_buf, 1};
    return StartTransfer(nullptr, &rx_set);
  }

  void ChipSelect(SpiPacket &pkt, bool enable) {
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

}  // namespace myl_utils
