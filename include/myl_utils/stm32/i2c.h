#pragma once

/**
 * @file i2c.h
 * @brief STM32 HAL I2C device implementation
 *
 * Provides synchronous and asynchronous I2C device classes for STM32 using the
 * HAL driver API. These implementations extend the I2c/AsyncI2c base classes
 * to work with the peripheral packet system.
 *
 * @note STM32 HAL uses 8-bit I2C addresses (left-shifted by 1). The addr field
 * in I2cPacket should contain the 7-bit address — this class shifts it
 * automatically before passing to HAL.
 *
 * Usage (synchronous):
 * @code
 * extern I2C_HandleTypeDef hi2c1;
 * Stm32I2cDevice i2c(&hi2c1);
 *
 * // Wrap transport with per-device config (address, callback)
 * I2cDevice<Stm32I2cDevice> eeprom(i2c, 0x50);
 *
 * // Create and configure buffers
 * Buffer<16> tx;
 * Buffer<16> rx;
 * tx.Set(0x00);  // Register address, length = 1
 * rx.length = 8;
 *
 * // Execute transfer — address is applied automatically
 * eeprom.WriteThenRead(tx, rx);
 * @endcode
 *
 * Usage (asynchronous):
 * @code
 * extern I2C_HandleTypeDef hi2c1;
 * Stm32AsyncI2cDevice i2c(&hi2c1);
 *
 * I2cDevice<Stm32AsyncI2cDevice<>> eeprom(i2c, 0x50, my_callback);
 *
 * I2cPacketBundle<16> cmd;
 * cmd.tx.Set(0x00);
 * cmd.rx.length = 8;
 * cmd.pkt.type = PacketType::WriteThenRead;
 * eeprom.ProcessCommand(cmd.pkt);
 *
 * // In your HAL callbacks (HAL_I2C_MasterTxCpltCallback, etc.):
 * i2c.TxRxCmpltCb();
 * @endcode
 */

#include <myl_utils/peripheral.h>

// Requires STM32 HAL to be included before this header.
// Typically satisfied by including your project's "main.h" or the HAL
// umbrella header (e.g. "stm32f4xx_hal.h") before this file.
#ifndef HAL_I2C_MODULE_ENABLED
#error "Include your STM32 HAL header (e.g. stm32f4xx_hal.h) before myl_utils/stm32/i2c.h"
#endif

namespace myl_utils {

/**
 * @brief Synchronous STM32 HAL I2C device implementation
 *
 * Uses HAL_I2C_Master_Transmit / HAL_I2C_Master_Receive /
 * HAL_I2C_Mem_Read for blocking transfers. Timeout is configurable
 * via the constructor.
 */
class Stm32I2cDevice : public I2c<Stm32I2cDevice> {
  friend class SyncPacketSender<Stm32I2cDevice, I2cPacket>;

 protected:
  I2C_HandleTypeDef *hi2c_;
  uint32_t timeout_;
  HAL_StatusTypeDef last_status_{HAL_OK};

  /// Helper: convert 7-bit address to HAL 8-bit format
  static uint16_t HalAddr(uint8_t addr) { return static_cast<uint16_t>(addr) << 1; }

  /**
   * @brief Write-then-read using HAL_I2C_Mem_Read
   *
   * Treats the TX buffer contents as the memory/register address and performs
   * a combined write+read with a repeated START. This is the idiomatic STM32
   * HAL pattern for register reads.
   *
   * Falls back to separate Transmit + Receive when TX length > 2 bytes
   * (exceeds HAL_I2C_Mem_Read's MemAddSize).
   */
  void ReadWritePacket(I2cPacket &pkt) {
    uint16_t addr = HalAddr(pkt.addr);
    uint16_t tx_len = pkt.tx_data->length;
    if (tx_len <= 2) {
      // Pack register address for Mem_Read (supports 1- or 2-byte reg addr)
      uint16_t mem_addr = pkt.tx_data->data[0];
      uint16_t mem_size = I2C_MEMADD_SIZE_8BIT;
      if (tx_len == 2) {
        mem_addr = (static_cast<uint16_t>(pkt.tx_data->data[0]) << 8) | pkt.tx_data->data[1];
        mem_size = I2C_MEMADD_SIZE_16BIT;
      }
      last_status_ = HAL_I2C_Mem_Read(
          hi2c_, addr, mem_addr, mem_size,
          pkt.rx_data->data, pkt.rx_data->length, timeout_);
    } else {
      // Fallback: manual write then read
      last_status_ = HAL_I2C_Master_Transmit(
          hi2c_, addr, pkt.tx_data->data, tx_len, timeout_);
      if (last_status_ == HAL_OK) {
        last_status_ = HAL_I2C_Master_Receive(
            hi2c_, addr, pkt.rx_data->data, pkt.rx_data->length, timeout_);
      }
    }
  }

  void WritePacket(I2cPacket &pkt) {
    last_status_ = HAL_I2C_Master_Transmit(
        hi2c_, HalAddr(pkt.addr),
        pkt.tx_data->data, pkt.tx_data->length, timeout_);
  }

  void ReadPacket(I2cPacket &pkt) {
    last_status_ = HAL_I2C_Master_Receive(
        hi2c_, HalAddr(pkt.addr),
        pkt.rx_data->data, pkt.rx_data->length, timeout_);
  }

  void ChipSelect(I2cPacket & /*pkt*/, bool /*enable*/) {
    // I2C does not use chip select — addressing is handled via the packet address
  }

 public:
  /**
   * @brief Construct a new STM32 I2C Device
   * @param hi2c Pointer to the HAL I2C handle (e.g. &hi2c1)
   * @param timeout_ms Timeout in milliseconds for blocking transfers (default: 100)
   */
  explicit Stm32I2cDevice(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms = 100)
      : hi2c_(hi2c), timeout_(timeout_ms) {}

  /// Get the HAL status from the last transfer
  HAL_StatusTypeDef last_status() const { return last_status_; }
};

/**
 * @brief Asynchronous (interrupt) STM32 HAL I2C device implementation
 *
 * Uses HAL_I2C_Master_Transmit_IT / HAL_I2C_Master_Receive_IT /
 * HAL_I2C_Mem_Read_IT for non-blocking transfers. Call TxRxCmpltCb() from the
 * appropriate HAL callbacks (HAL_I2C_MasterTxCpltCallback,
 * HAL_I2C_MasterRxCpltCallback, HAL_I2C_MemRxCpltCallback) to advance the
 * transfer queue.
 *
 * @tparam QueueSize Maximum number of packets that can be queued (default: 25)
 */
template <uint8_t QueueSize = 32>
class Stm32AsyncI2cDevice : public AsyncI2c<Stm32AsyncI2cDevice<QueueSize>, QueueSize> {
  friend class AsyncPacketSender<Stm32AsyncI2cDevice<QueueSize>, I2cPacket, QueueSize>;

 protected:
  I2C_HandleTypeDef *hi2c_;
  HAL_StatusTypeDef last_status_{HAL_OK};

  /// Helper: convert 7-bit address to HAL 8-bit format
  static uint16_t HalAddr(uint8_t addr) { return static_cast<uint16_t>(addr) << 1; }

  /**
   * @brief Write-then-read using HAL_I2C_Mem_Read_IT
   *
   * Falls back to separate Transmit_IT + Receive_IT when TX length > 2 bytes.
   * When falling back, the read phase is handled by the AsyncPacketSender's
   * WriteThenRead sequencing via TxRxCmpltCb().
   */
  void ReadWritePacket(I2cPacket &pkt) {
    uint16_t addr = HalAddr(pkt.addr);
    uint16_t tx_len = pkt.tx_data->length;
    if (tx_len <= 2) {
      uint16_t mem_addr = pkt.tx_data->data[0];
      uint16_t mem_size = I2C_MEMADD_SIZE_8BIT;
      if (tx_len == 2) {
        mem_addr = (static_cast<uint16_t>(pkt.tx_data->data[0]) << 8) | pkt.tx_data->data[1];
        mem_size = I2C_MEMADD_SIZE_16BIT;
      }
      last_status_ = HAL_I2C_Mem_Read_IT(
          hi2c_, addr, mem_addr, mem_size,
          pkt.rx_data->data, pkt.rx_data->length);
    } else {
      // The base class WriteThenRead sequencing handles the read phase
      last_status_ = HAL_I2C_Master_Transmit_IT(
          hi2c_, addr, pkt.tx_data->data, tx_len);
    }
  }

  void WritePacket(I2cPacket &pkt) {
    last_status_ = HAL_I2C_Master_Transmit_IT(
        hi2c_, HalAddr(pkt.addr),
        pkt.tx_data->data, pkt.tx_data->length);
  }

  void ReadPacket(I2cPacket &pkt) {
    last_status_ = HAL_I2C_Master_Receive_IT(
        hi2c_, HalAddr(pkt.addr),
        pkt.rx_data->data, pkt.rx_data->length);
  }

  void ChipSelect(I2cPacket & /*pkt*/, bool /*enable*/) {
    // I2C does not use chip select
  }

 public:
  /**
   * @brief Construct a new STM32 Async I2C Device
   * @param hi2c Pointer to the HAL I2C handle (e.g. &hi2c1)
   */
  explicit Stm32AsyncI2cDevice(I2C_HandleTypeDef *hi2c) : hi2c_(hi2c) {}

  /// Get the HAL status from the last transfer
  HAL_StatusTypeDef last_status() const { return last_status_; }
};

}  // namespace myl_utils
