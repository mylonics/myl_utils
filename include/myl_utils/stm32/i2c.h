#pragma once

/**
 * @file i2c.h
 * @brief STM32 HAL I2C unified transport (sync + async in one class)
 *
 * Provides a single I2C transport class for STM32 that supports both blocking
 * (sync) and non-blocking (interrupt/DMA) transfers. Inherits from both
 * SyncI2c and AsyncI2c CRTP bases, so a single I2cDevice wrapper can call
 * SendSync() and SendAsync() on the same transport object.
 *
 * @note STM32 HAL uses 8-bit I2C addresses (left-shifted by 1). The addr field
 * in I2cPacket should contain the 7-bit address — this class shifts it
 * automatically before passing to HAL.
 *
 * Usage (sync):
 * @code
 * extern I2C_HandleTypeDef hi2c1;
 * Stm32I2cTransport<> i2c(&hi2c1);
 *
 * I2cDevice<Stm32I2cTransport<>> eeprom(i2c, 0x50);
 *
 * Buffer<16> tx, rx;
 * tx.Set(0x00);
 * rx.length = 8;
 * auto pkt = I2cPacket::RegRead(tx, rx);
 * eeprom.SendSync(pkt);
 * @endcode
 *
 * Usage (async, interrupt):
 * @code
 * extern I2C_HandleTypeDef hi2c1;
 * Stm32I2cTransport<> i2c(&hi2c1);
 *
 * I2cDevice<Stm32I2cTransport<>> eeprom(i2c, 0x50, my_callback);
 *
 * I2cPacketBundle<16> cmd;
 * cmd.tx.Set(0x00);
 * cmd.rx.length = 8;
 * cmd.pkt.type = PacketType::WriteThenRead;
 * eeprom.SendAsync(cmd.pkt);
 *
 * // In your HAL callbacks (HAL_I2C_MasterTxCpltCallback, etc.):
 * i2c.TxRxCmpltCb();
 * @endcode
 *
 * Usage (async, DMA):
 * @code
 * extern I2C_HandleTypeDef hi2c1;
 * Stm32I2cTransport<TransferMode::Dma> i2c(&hi2c1);
 *
 * I2cDevice<Stm32I2cTransport<TransferMode::Dma>> eeprom(i2c, 0x50, my_callback);
 *
 * // Buffers must be in DMA-accessible RAM (not DTCM on F7/H7)
 * I2cPacketBundle<16> cmd;
 * cmd.tx.Set(0x00);
 * cmd.rx.length = 8;
 * cmd.pkt.type = PacketType::WriteThenRead;
 * eeprom.SendAsync(cmd.pkt);
 *
 * // In your HAL DMA callbacks:
 * i2c.TxRxCmpltCb();
 * @endcode
 *
 * Usage (mixed sync + async):
 * @code
 * Stm32I2cTransport<TransferMode::Dma> i2c(&hi2c1);
 * I2cDevice<Stm32I2cTransport<TransferMode::Dma>> sensor(i2c, 0x68, my_callback);
 *
 * // Blocking register read during init
 * auto pkt = I2cPacket::RegRead(tx, rx);
 * sensor.SendSync(pkt);
 *
 * // Non-blocking DMA transfer during runtime
 * sensor.SendAsync(bulk.pkt);
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
 * @brief Unified STM32 HAL I2C transport (sync + async)
 *
 * Provides both blocking (HAL_I2C_Master_Transmit, etc.) and non-blocking
 * (HAL_I2C_Master_Transmit_IT / _DMA) transfers through a single object. The
 * sync path uses timeout-based blocking HAL calls; the async path uses the
 * AsyncPacketSender queue and requires TxRxCmpltCb() to be called from HAL
 * interrupt callbacks.
 *
 * @note When using DMA mode, all TX/RX buffers for async transfers must reside
 * in DMA-accessible memory. On STM32 F7/H7, this excludes DTCM/ITCM.
 *
 * @tparam Mode Async transfer mode: TransferMode::Interrupt (default) or TransferMode::Dma
 * @tparam QueueSize Maximum number of async packets that can be queued (default: 32)
 */
template <TransferMode Mode = TransferMode::Interrupt, uint8_t QueueSize = 32>
class Stm32I2cTransport
    : public SyncI2c<Stm32I2cTransport<Mode, QueueSize>>
    , public AsyncI2c<Stm32I2cTransport<Mode, QueueSize>, QueueSize> {
  friend class SyncPacketSender<Stm32I2cTransport<Mode, QueueSize>, I2cPacket>;
  friend class AsyncPacketSender<Stm32I2cTransport<Mode, QueueSize>, I2cPacket, QueueSize>;

 protected:
  I2C_HandleTypeDef *hi2c_;
  uint32_t timeout_;
  uint32_t i2c_clock_hz_;
  HAL_StatusTypeDef last_status_{HAL_OK};

  /// Helper: convert 7-bit address to HAL 8-bit format
  static uint16_t HalAddr(uint8_t addr) { return static_cast<uint16_t>(addr) << 1; }

  // ---- Shared chip select (I2C has none) ---------------------------------

  MYL_NOINLINE void ChipSelect(I2cPacket & /*pkt*/, bool /*enable*/) {
    // I2C does not use chip select — addressing is handled via the packet address
  }

  MYL_NOINLINE void ChipSelectFast(I2cPacket & /*pkt*/, bool /*enable*/) {
    // I2C has no chip select
  }

  // ---- Synchronous (blocking) implementations ---------------------------

  /**
   * @brief Sync write-then-read using HAL_I2C_Mem_Read
   *
   * Treats the TX buffer contents as the memory/register address and performs
   * a combined write+read with a repeated START. Falls back to separate
   * Transmit + Receive when TX length > 2 bytes.
   */
  MYL_NOINLINE bool SyncReadWritePacket(I2cPacket &pkt) {
    uint16_t addr = HalAddr(pkt.addr);
    uint16_t tx_len = pkt.tx_data->length;
    if (tx_len <= 2) {
      uint16_t mem_addr = pkt.tx_data->data[0];
      uint16_t mem_size = I2C_MEMADD_SIZE_8BIT;
      if (tx_len == 2) {
        mem_addr = (static_cast<uint16_t>(pkt.tx_data->data[0]) << 8) | pkt.tx_data->data[1];
        mem_size = I2C_MEMADD_SIZE_16BIT;
      }
      last_status_ = HAL_I2C_Mem_Read(
          hi2c_, addr, mem_addr, mem_size,
          pkt.rx_data->data, pkt.rx_data->length, ComputeSyncTimeout(pkt));
    } else {
      last_status_ = HAL_I2C_Master_Transmit(
          hi2c_, addr, pkt.tx_data->data, tx_len, ComputeSyncTimeout(pkt));
      if (last_status_ == HAL_OK) {
        last_status_ = HAL_I2C_Master_Receive(
            hi2c_, addr, pkt.rx_data->data, pkt.rx_data->length, ComputeSyncTimeout(pkt));
      }
    }
    return last_status_ == HAL_OK;
  }

  MYL_NOINLINE bool SyncWritePacket(I2cPacket &pkt) {
    last_status_ = HAL_I2C_Master_Transmit(
        hi2c_, HalAddr(pkt.addr),
        pkt.tx_data->data, pkt.tx_data->length, ComputeSyncTimeout(pkt));
    return last_status_ == HAL_OK;
  }

  MYL_NOINLINE bool SyncReadPacket(I2cPacket &pkt) {
    last_status_ = HAL_I2C_Master_Receive(
        hi2c_, HalAddr(pkt.addr),
        pkt.rx_data->data, pkt.rx_data->length, ComputeSyncTimeout(pkt));
    return last_status_ == HAL_OK;
  }

  // ---- Asynchronous (interrupt/DMA) implementations ---------------------

  /**
   * @brief Async write-then-read using HAL_I2C_Mem_Read_IT / _DMA
   *
   * Falls back to separate Transmit_IT + Receive_IT when TX length > 2 bytes.
   * When falling back, the read phase is handled by the AsyncPacketSender's
   * WriteThenRead sequencing via TxRxCmpltCb().
   */
  MYL_NOINLINE bool AsyncReadWritePacket(I2cPacket &pkt) {
    uint16_t addr = HalAddr(pkt.addr);
    uint16_t tx_len = pkt.tx_data->length;
    if (tx_len <= 2) {
      uint16_t mem_addr = pkt.tx_data->data[0];
      uint16_t mem_size = I2C_MEMADD_SIZE_8BIT;
      if (tx_len == 2) {
        mem_addr = (static_cast<uint16_t>(pkt.tx_data->data[0]) << 8) | pkt.tx_data->data[1];
        mem_size = I2C_MEMADD_SIZE_16BIT;
      }
      if constexpr (Mode == TransferMode::Dma) {
        last_status_ = HAL_I2C_Mem_Read_DMA(
            hi2c_, addr, mem_addr, mem_size,
            pkt.rx_data->data, pkt.rx_data->length);
      } else {
        last_status_ = HAL_I2C_Mem_Read_IT(
            hi2c_, addr, mem_addr, mem_size,
            pkt.rx_data->data, pkt.rx_data->length);
      }
    } else {
      if constexpr (Mode == TransferMode::Dma) {
        last_status_ = HAL_I2C_Master_Transmit_DMA(
            hi2c_, addr, pkt.tx_data->data, tx_len);
      } else {
        last_status_ = HAL_I2C_Master_Transmit_IT(
            hi2c_, addr, pkt.tx_data->data, tx_len);
      }
    }
    return last_status_ == HAL_OK;
  }

  MYL_NOINLINE bool AsyncWritePacket(I2cPacket &pkt) {
    if constexpr (Mode == TransferMode::Dma) {
      last_status_ = HAL_I2C_Master_Transmit_DMA(
          hi2c_, HalAddr(pkt.addr),
          pkt.tx_data->data, pkt.tx_data->length);
    } else {
      last_status_ = HAL_I2C_Master_Transmit_IT(
          hi2c_, HalAddr(pkt.addr),
          pkt.tx_data->data, pkt.tx_data->length);
    }
    return last_status_ == HAL_OK;
  }

  MYL_NOINLINE bool AsyncReadPacket(I2cPacket &pkt) {
    if constexpr (Mode == TransferMode::Dma) {
      last_status_ = HAL_I2C_Master_Receive_DMA(
          hi2c_, HalAddr(pkt.addr),
          pkt.rx_data->data, pkt.rx_data->length);
    } else {
      last_status_ = HAL_I2C_Master_Receive_IT(
          hi2c_, HalAddr(pkt.addr),
          pkt.rx_data->data, pkt.rx_data->length);
    }
    return last_status_ == HAL_OK;
  }

 public:
  /**
   * @brief Construct a new STM32 I2C Transport
   * @param hi2c Pointer to the HAL I2C handle (e.g. &hi2c1)
   * @param timeout_ms Fallback timeout in ms for sync transfers when i2c_clock_hz is 0 (default: 100)
   * @param i2c_clock_hz I2C bus clock in Hz for auto-computed timeouts (e.g. 100000, 400000).
   *                     0 = unknown; falls back to timeout_ms for all sync transfers.
   *                     Use Init.ClockSpeed on F1/F4 or pass the known bus speed on G4/H7.
   */
  explicit Stm32I2cTransport(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms = 100,
                              uint32_t i2c_clock_hz = 0)
      : hi2c_(hi2c), timeout_(timeout_ms), i2c_clock_hz_(i2c_clock_hz) {}

  /// Get the HAL status from the last transfer (sync or async)
  HAL_StatusTypeDef last_status() const { return last_status_; }

 private:
  /**
   * @brief Compute a physically appropriate sync timeout for a given packet
   *
   * When i2c_clock_hz_ is provided, derives the timeout from the bus speed and
   * transfer length with a 2x safety factor. Falls back to timeout_ otherwise.
   *
   * Priority: pkt.timeout_ms (explicit) > auto-computed > timeout_ (fallback)
   */
  uint32_t ComputeSyncTimeout(const I2cPacket &pkt) const {
    if (pkt.timeout_ms > 0) return pkt.timeout_ms;
    if (i2c_clock_hz_ > 0) {
      uint16_t bytes = 0;
      if (pkt.tx_data) bytes += pkt.tx_data->length;
      if (pkt.rx_data) bytes += pkt.rx_data->length;
      // 9 bits/byte (8 data + ACK), +2 for START/address/STOP overhead
      // 2× safety factor × 1000 ms/s: (bytes+2) × 9 × 2 × 1000 / clock_hz + 1
      return (static_cast<uint32_t>(bytes + 2u) * 18000u / i2c_clock_hz_) + 1u;
    }
    return timeout_;
  }
};

/// @brief Convenience alias for DMA-mode I2C transport
/// @tparam QueueSize Maximum number of async packets that can be queued (default: 32)
template <uint8_t QueueSize = 32>
using Stm32DmaI2cTransport = Stm32I2cTransport<TransferMode::Dma, QueueSize>;

}  // namespace myl_utils
