#pragma once

/**
 * @file spi.h
 * @brief STM32 HAL SPI device implementation
 *
 * Provides synchronous and asynchronous SPI device classes for STM32 using the
 * HAL driver API. These implementations extend the Spi/AsyncSpi base classes
 * to work with the peripheral packet system.
 *
 * Usage (synchronous):
 * @code
 * extern SPI_HandleTypeDef hspi1;
 * Stm32SpiDevice spi(&hspi1);
 *
 * // Chip select pin (configured by CubeMX)
 * Stm32GpioOutput cs_pin(CS_GPIO_Port, CS_Pin);
 *
 * // Wrap transport with per-device config (chip select, callback)
 * SpiDevice<Stm32SpiDevice> sensor(spi, cs_pin);
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
 *
 * Usage (asynchronous):
 * @code
 * extern SPI_HandleTypeDef hspi1;
 * Stm32AsyncSpiDevice spi(&hspi1);
 *
 * Stm32GpioOutput cs_pin(CS_GPIO_Port, CS_Pin);
 * SpiDevice<Stm32AsyncSpiDevice<>> sensor(spi, cs_pin, my_callback);
 *
 * SpiPacketBundle<16> cmd;
 * cmd.tx.Set(0x80 | REG_ADDR);
 * cmd.rx.length = 4;
 * cmd.pkt.type = PacketType::WriteThenRead;
 * sensor.ProcessCommand(cmd.pkt);
 *
 * // In your HAL callback (e.g. HAL_SPI_TxCpltCallback):
 * spi.TxRxCmpltCb();
 * @endcode
 */

#include <myl_utils/peripheral.h>

// Requires STM32 HAL to be included before this header.
// Typically satisfied by including your project's "main.h" or the HAL
// umbrella header (e.g. "stm32f4xx_hal.h") before this file.
#ifndef HAL_SPI_MODULE_ENABLED
#error "Include your STM32 HAL header (e.g. stm32f4xx_hal.h) before myl_utils/stm32/spi.h"
#endif

/**
 * @brief Synchronous STM32 HAL SPI device implementation
 *
 * Uses HAL_SPI_Transmit / HAL_SPI_Receive / HAL_SPI_TransmitReceive for
 * blocking transfers. Timeout is configurable via the constructor.
 */
class Stm32SpiDevice : public Spi<Stm32SpiDevice> {
  friend class SyncPacketSender<Stm32SpiDevice, SpiPacket>;

 protected:
  SPI_HandleTypeDef *hspi_;
  uint32_t timeout_;
  HAL_StatusTypeDef last_status_{HAL_OK};

  void ReadWritePacket(SpiPacket &pkt) {
    last_status_ = HAL_SPI_TransmitReceive(
        hspi_, pkt.tx_data->data, pkt.rx_data->data,
        pkt.tx_data->length, timeout_);
  }

  void WritePacket(SpiPacket &pkt) {
    last_status_ = HAL_SPI_Transmit(
        hspi_, pkt.tx_data->data, pkt.tx_data->length, timeout_);
  }

  void ReadPacket(SpiPacket &pkt) {
    last_status_ = HAL_SPI_Receive(
        hspi_, pkt.rx_data->data, pkt.rx_data->length, timeout_);
  }

  void ChipSelect(SpiPacket &pkt, bool enable) {
    if (enable) {
      uint32_t target_cpol = (pkt.polarity == SpiPolarity::High) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
      uint32_t target_cpha = (pkt.phase == SpiPhase::Trailing) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
      if (hspi_->Init.CLKPolarity != target_cpol || hspi_->Init.CLKPhase != target_cpha) {
        hspi_->Init.CLKPolarity = target_cpol;
        hspi_->Init.CLKPhase = target_cpha;
        HAL_SPI_Init(hspi_);
      }
    }
    pkt.chip_select.Set(enable);
  }

 public:
  /**
   * @brief Construct a new STM32 SPI Device
   * @param hspi Pointer to the HAL SPI handle (e.g. &hspi1)
   * @param timeout_ms Timeout in milliseconds for blocking transfers (default: 100)
   */
  explicit Stm32SpiDevice(SPI_HandleTypeDef *hspi, uint32_t timeout_ms = 100)
      : hspi_(hspi), timeout_(timeout_ms) {}

  /// Get the HAL status from the last transfer
  HAL_StatusTypeDef last_status() const { return last_status_; }
};

/**
 * @brief Asynchronous (interrupt/DMA) STM32 HAL SPI device implementation
 *
 * Uses HAL_SPI_Transmit_IT / HAL_SPI_Receive_IT / HAL_SPI_TransmitReceive_IT
 * for non-blocking transfers. Call TxRxCmpltCb() from the appropriate HAL
 * callbacks (HAL_SPI_TxCpltCallback, HAL_SPI_RxCpltCallback,
 * HAL_SPI_TxRxCpltCallback) to advance the transfer queue.
 *
 * @tparam QueueSize Maximum number of packets that can be queued (default: 25)
 */
template <uint8_t QueueSize = 32>
class Stm32AsyncSpiDevice : public AsyncSpi<Stm32AsyncSpiDevice<QueueSize>, QueueSize> {
  friend class AsyncPacketSender<Stm32AsyncSpiDevice<QueueSize>, SpiPacket, QueueSize>;

 protected:
  SPI_HandleTypeDef *hspi_;
  HAL_StatusTypeDef last_status_{HAL_OK};

  void ReadWritePacket(SpiPacket &pkt) {
    last_status_ = HAL_SPI_TransmitReceive_IT(
        hspi_, pkt.tx_data->data, pkt.rx_data->data,
        pkt.tx_data->length);
  }

  void WritePacket(SpiPacket &pkt) {
    last_status_ = HAL_SPI_Transmit_IT(
        hspi_, pkt.tx_data->data, pkt.tx_data->length);
  }

  void ReadPacket(SpiPacket &pkt) {
    last_status_ = HAL_SPI_Receive_IT(
        hspi_, pkt.rx_data->data, pkt.rx_data->length);
  }

  void ChipSelect(SpiPacket &pkt, bool enable) {
    if (enable) {
      uint32_t target_cpol = (pkt.polarity == SpiPolarity::High) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
      uint32_t target_cpha = (pkt.phase == SpiPhase::Trailing) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
      if (hspi_->Init.CLKPolarity != target_cpol || hspi_->Init.CLKPhase != target_cpha) {
        hspi_->Init.CLKPolarity = target_cpol;
        hspi_->Init.CLKPhase = target_cpha;
        HAL_SPI_Init(hspi_);
      }
    }
    pkt.chip_select.Set(enable);
  }

 public:
  /**
   * @brief Construct a new STM32 Async SPI Device
   * @param hspi Pointer to the HAL SPI handle (e.g. &hspi1)
   */
  explicit Stm32AsyncSpiDevice(SPI_HandleTypeDef *hspi) : hspi_(hspi) {}

  /// Get the HAL status from the last transfer
  HAL_StatusTypeDef last_status() const { return last_status_; }
};
