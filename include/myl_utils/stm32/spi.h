#pragma once

/**
 * @file spi.h
 * @brief STM32 HAL SPI unified transport (sync + async in one class)
 *
 * Provides a single SPI transport class for STM32 that supports both blocking
 * (sync) and non-blocking (interrupt/DMA) transfers. Inherits from both
 * SyncSpi and AsyncSpi CRTP bases, so a single SpiDevice wrapper can call
 * SendSync() and SendAsync() on the same transport object.
 *
 * Usage (sync):
 * @code
 * extern SPI_HandleTypeDef hspi1;
 * Stm32SpiTransport<> spi(&hspi1);
 *
 * Stm32GpioOutput cs_gpio(CS_GPIO_Port, CS_Pin);
 * SpiDevice<Stm32SpiTransport<>> sensor(spi, ChipSelectPin(cs_gpio));
 *
 * Buffer<16> tx, rx;
 * tx.Set(0x80 | REG_ADDR);
 * rx.length = 4;
 * auto pkt = SpiPacket::RegRead(tx, rx);
 * sensor.SendSync(pkt);
 * @endcode
 *
 * Usage (async, interrupt):
 * @code
 * extern SPI_HandleTypeDef hspi1;
 * Stm32SpiTransport<> spi(&hspi1);
 *
 * SpiDevice<Stm32SpiTransport<>> sensor(spi, ChipSelectPin(cs_gpio), my_callback);
 *
 * SpiPacketBundle<16> cmd;
 * cmd.tx.Set(0x80 | REG_ADDR);
 * cmd.rx.length = 4;
 * cmd.pkt.type = PacketType::WriteThenRead;
 * sensor.SendAsync(cmd.pkt);
 *
 * // In your HAL callback (e.g. HAL_SPI_TxCpltCallback):
 * spi.TxRxCmpltCb();
 * @endcode
 *
 * Usage (async, DMA):
 * @code
 * extern SPI_HandleTypeDef hspi1;
 * Stm32SpiTransport<TransferMode::Dma> spi(&hspi1);
 *
 * SpiDevice<Stm32SpiTransport<TransferMode::Dma>> sensor(spi, ChipSelectPin(cs_gpio), my_callback);
 *
 * // Buffers must be in DMA-accessible RAM (not DTCM on F7/H7)
 * SpiPacketBundle<16> cmd;
 * cmd.tx.Set(0x80 | REG_ADDR);
 * cmd.rx.length = 4;
 * cmd.pkt.type = PacketType::WriteThenRead;
 * sensor.SendAsync(cmd.pkt);
 *
 * // In your HAL DMA callback (HAL_SPI_TxCpltCallback, etc.):
 * spi.TxRxCmpltCb();
 * @endcode
 *
 * Usage (mixed sync + async on same device):
 * @code
 * Stm32SpiTransport<TransferMode::Dma> spi(&hspi1);
 * SpiDevice<Stm32SpiTransport<TransferMode::Dma>> sensor(spi, ChipSelectPin(cs_gpio), my_callback);
 *
 * // Blocking single-register read during init
 * auto pkt = SpiPacket::RegRead(tx, rx);
 * sensor.SendSync(pkt);
 *
 * // Non-blocking DMA bulk read during runtime
 * static SpiPacketBundle<256> bulk;
 * bulk.pkt.type = PacketType::Read;
 * bulk.rx.length = 256;
 * sensor.SendAsync(bulk.pkt);
 * @endcode
 */

#include <myl_utils/peripheral.h>

// Requires STM32 HAL to be included before this header.
// Typically satisfied by including your project's "main.h" or the HAL
// umbrella header (e.g. "stm32f4xx_hal.h") before this file.
#ifndef HAL_SPI_MODULE_ENABLED
#error "Include your STM32 HAL header (e.g. stm32f4xx_hal.h) before myl_utils/stm32/spi.h"
#endif

namespace myl_utils {

/**
 * @brief Unified STM32 HAL SPI transport (sync + async)
 *
 * Provides both blocking (HAL_SPI_Transmit, etc.) and non-blocking
 * (HAL_SPI_Transmit_IT / _DMA) transfers through a single object. The sync
 * path has no queue or ISR overhead; the async path uses the AsyncPacketSender
 * queue and requires TxRxCmpltCb() to be called from HAL interrupt callbacks.
 *
 * @note When using DMA mode, all TX/RX buffers for async transfers must reside
 * in DMA-accessible memory. On STM32 F7/H7, this excludes DTCM/ITCM.
 * Sync transfers have no such restriction.
 *
 * @tparam Mode Async transfer mode: TransferMode::Interrupt (default) or TransferMode::Dma
 * @tparam QueueSize Maximum number of async packets that can be queued (default: 32)
 */
template <TransferMode Mode = TransferMode::Interrupt, uint8_t QueueSize = 32>
class Stm32SpiTransport
    : public SyncSpi<Stm32SpiTransport<Mode, QueueSize>>
    , public AsyncSpi<Stm32SpiTransport<Mode, QueueSize>, QueueSize> {
  friend class SyncPacketSender<Stm32SpiTransport<Mode, QueueSize>, SpiPacket>;
  friend class AsyncPacketSender<Stm32SpiTransport<Mode, QueueSize>, SpiPacket, QueueSize>;

 protected:
  SPI_HandleTypeDef *hspi_;
  uint32_t timeout_;
  uint32_t pclk_hz_{};
  uint32_t default_prescaler_{};
  HAL_StatusTypeDef last_status_{HAL_OK};

  // ---- Shared chip select (same for sync and async paths) ----------------

  MYL_NOINLINE void ChipSelect(SpiPacket &pkt, bool enable) {
    if (enable) {
      bool need_reinit = false;

      uint32_t target_cpol = (pkt.polarity == SpiPolarity::High) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
      uint32_t target_cpha = (pkt.phase == SpiPhase::Trailing) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
      if (hspi_->Init.CLKPolarity != target_cpol || hspi_->Init.CLKPhase != target_cpha) {
        hspi_->Init.CLKPolarity = target_cpol;
        hspi_->Init.CLKPhase = target_cpha;
        need_reinit = true;
      }

      // Compute target baud-rate prescaler (override or default)
      uint32_t target_br = default_prescaler_;
      if (pkt.max_freq_hz > 0 && pclk_hz_ > 0) {
        uint32_t computed = ComputeBaudRatePrescaler(pkt.max_freq_hz);
        // Only slow down — never speed up beyond the configured default
        if (computed > default_prescaler_) target_br = computed;
      }
      if (hspi_->Init.BaudRatePrescaler != target_br) {
        hspi_->Init.BaudRatePrescaler = target_br;
        need_reinit = true;
      }

      if (need_reinit) HAL_SPI_Init(hspi_);
    }
    pkt.chip_select.Set(enable);
  }

  // ---- Synchronous (blocking) implementations ---------------------------

  MYL_NOINLINE bool SyncReadWritePacket(SpiPacket &pkt) {
    last_status_ = HAL_SPI_TransmitReceive(
        hspi_, pkt.tx_data->data, pkt.rx_data->data,
        pkt.tx_data->length, timeout_);
    return last_status_ == HAL_OK;
  }

  MYL_NOINLINE bool SyncWritePacket(SpiPacket &pkt) {
    last_status_ = HAL_SPI_Transmit(
        hspi_, pkt.tx_data->data, pkt.tx_data->length, timeout_);
    return last_status_ == HAL_OK;
  }

  MYL_NOINLINE bool SyncReadPacket(SpiPacket &pkt) {
    last_status_ = HAL_SPI_Receive(
        hspi_, pkt.rx_data->data, pkt.rx_data->length, timeout_);
    return last_status_ == HAL_OK;
  }

  // ---- Asynchronous (interrupt/DMA) implementations ---------------------

  MYL_NOINLINE bool AsyncReadWritePacket(SpiPacket &pkt) {
    if constexpr (Mode == TransferMode::Dma) {
      last_status_ = HAL_SPI_TransmitReceive_DMA(
          hspi_, pkt.tx_data->data, pkt.rx_data->data,
          pkt.tx_data->length);
    } else {
      last_status_ = HAL_SPI_TransmitReceive_IT(
          hspi_, pkt.tx_data->data, pkt.rx_data->data,
          pkt.tx_data->length);
    }
    return last_status_ == HAL_OK;
  }

  MYL_NOINLINE bool AsyncWritePacket(SpiPacket &pkt) {
    if constexpr (Mode == TransferMode::Dma) {
      last_status_ = HAL_SPI_Transmit_DMA(
          hspi_, pkt.tx_data->data, pkt.tx_data->length);
    } else {
      last_status_ = HAL_SPI_Transmit_IT(
          hspi_, pkt.tx_data->data, pkt.tx_data->length);
    }
    return last_status_ == HAL_OK;
  }

  MYL_NOINLINE bool AsyncReadPacket(SpiPacket &pkt) {
    if constexpr (Mode == TransferMode::Dma) {
      last_status_ = HAL_SPI_Receive_DMA(
          hspi_, pkt.rx_data->data, pkt.rx_data->length);
    } else {
      last_status_ = HAL_SPI_Receive_IT(
          hspi_, pkt.rx_data->data, pkt.rx_data->length);
    }
    return last_status_ == HAL_OK;
  }

 public:
  /**
   * @brief Construct a new STM32 SPI Transport
   * @param hspi Pointer to the HAL SPI handle (e.g. &hspi1)
   * @param timeout_ms Timeout in milliseconds for blocking (sync) transfers (default: 100)
   * @param pclk_hz APB clock feeding this SPI peripheral, in Hz (0 = disable frequency override).
   *                For SPI1 (APB2): HAL_RCC_GetPCLK2Freq(), SPI2/3 (APB1): HAL_RCC_GetPCLK1Freq()
   */
  explicit Stm32SpiTransport(SPI_HandleTypeDef *hspi, uint32_t timeout_ms = 100, uint32_t pclk_hz = 0)
      : hspi_(hspi), timeout_(timeout_ms), pclk_hz_(pclk_hz),
        default_prescaler_(hspi->Init.BaudRatePrescaler) {}

  /// Get the HAL status from the last transfer (sync or async)
  HAL_StatusTypeDef last_status() const { return last_status_; }

 private:
  /// Compute HAL-compatible BaudRatePrescaler value for a target max frequency
  uint32_t ComputeBaudRatePrescaler(uint32_t target_hz) const {
    const uint32_t prescalers[] = {
        SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
        SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
        SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256};
    uint32_t divisor = 2;
    for (uint8_t i = 0; i < 7; i++) {
      if ((pclk_hz_ / divisor) <= target_hz) return prescalers[i];
      divisor <<= 1;
    }
    return SPI_BAUDRATEPRESCALER_256;
  }
};

/// @brief Convenience alias for DMA-mode SPI transport
/// @tparam QueueSize Maximum number of async packets that can be queued (default: 32)
template <uint8_t QueueSize = 32>
using Stm32DmaSpiTransport = Stm32SpiTransport<TransferMode::Dma, QueueSize>;

}  // namespace myl_utils
