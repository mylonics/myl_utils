#pragma once

/**
 * @file stm32/can.h
 * @brief STM32 FDCAN transport for CanBase<Derived> CRTP
 *
 * Wraps the STM32 HAL FDCAN API (HAL_FDCAN_*) for use with CanBase<Derived>.
 * Compatible with STM32G0, G4, H7, L5, U5 and other devices with the
 * FDCAN peripheral.
 *
 * @note For devices with the older bxCAN peripheral (STM32F1, F4, F7, L1)
 * use a separate Stm32CanTransport (HAL_CAN_*) — to be added in a future revision.
 *
 * Transmit: HAL_FDCAN_AddMessageToTxFifoQ() — non-blocking if TX FIFO has space.
 * Receive:  HAL_FDCAN_ActivateNotification() for FIFO0/FIFO1 interrupt;
 *           received frames are stored in an internal CircularBuffer and
 *           can be read via GetFrame() or dispatched via registered callbacks.
 *
 * ISR integration:
 *   Call can.RxFifo0MsgPendingCb() from HAL_FDCAN_RxFifo0Callback()
 *   Call can.RxFifo1MsgPendingCb() from HAL_FDCAN_RxFifo1Callback()
 *   Call can.TxFifoEmptyCb()       from HAL_FDCAN_TxFifoEmptyCallback()  (optional)
 *
 * Usage:
 * @code
 * // Include your STM32 HAL header first (e.g. stm32g4xx_hal.h)
 * #include <myl_utils/stm32/can.h>
 *
 * extern FDCAN_HandleTypeDef hfdcan1;
 *
 * myl_utils::Stm32FdCanTransport<> can(&hfdcan1);
 *
 * void app_init() {
 *   can.Start();
 *
 *   // Register filter + callback for ID 0x123
 *   myl_utils::CanFilter f{.id = 0x123, .mask = 0x7FF};
 *   can.AddRxFilter(f, [](const myl_utils::CanFrame &frm, void *) {
 *     // handle frame
 *   });
 *
 *   // Send a frame
 *   myl_utils::CanFrame tx;
 *   tx.id = 0x200; tx.dlc = 2; tx.data[0] = 0x01; tx.data[1] = 0x02;
 *   can.Send(tx);
 * }
 *
 * // In HAL callbacks:
 * void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h, uint32_t f) {
 *   if (h == &hfdcan1) can.RxFifo0MsgPendingCb();
 * }
 * void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *h, uint32_t f) {
 *   if (h == &hfdcan1) can.RxFifo1MsgPendingCb();
 * }
 * @endcode
 */

// Requires STM32 HAL to be included before this header.
#ifndef HAL_FDCAN_MODULE_ENABLED
#error "Include your STM32 HAL header (e.g. stm32g4xx_hal.h) before myl_utils/stm32/can.h"
#endif

#include <myl_utils/can.h>
#include <myl_utils/buffer.h>
#include <myl_utils/noncopyable.h>

namespace myl_utils {

/// Maximum number of RX filters per transport instance.
static constexpr uint8_t kFdCanMaxFilters = 8;

/**
 * @brief STM32 FDCAN transport.
 *
 * @tparam RxQueueSize  Depth of the internal RX frame circular buffer (power of 2)
 */
template <size_t RxQueueSize = 16>
class Stm32FdCanTransport
    : public CanBase<Stm32FdCanTransport<RxQueueSize>>,
      NonCopyable<Stm32FdCanTransport<RxQueueSize>> {
  friend class CanBase<Stm32FdCanTransport<RxQueueSize>>;

  static_assert((RxQueueSize & (RxQueueSize - 1)) == 0,
                "RxQueueSize must be a power of 2");

  static constexpr size_t kQMask = RxQueueSize - 1;

 public:
  explicit Stm32FdCanTransport(FDCAN_HandleTypeDef *hfdcan) : hfdcan_(hfdcan) {}

  // ---- ISR integration -------------------------------------------------

  /// Call from HAL_FDCAN_RxFifo0Callback()
  MYL_ISR_NOINLINE void RxFifo0MsgPendingCb() { DrainFifo(FDCAN_RX_FIFO0); }

  /// Call from HAL_FDCAN_RxFifo1Callback()
  MYL_ISR_NOINLINE void RxFifo1MsgPendingCb() { DrainFifo(FDCAN_RX_FIFO1); }

  /// Call from HAL_FDCAN_TxFifoEmptyCallback() (optional — for future TX queuing)
  MYL_ISR_NOINLINE void TxFifoEmptyCb() {}

  HAL_StatusTypeDef LastStatus() const { return last_status_; }

 protected:
  // ---- CanBase<Derived> interface --------------------------------------

  MYL_NOINLINE bool ImplStart() {
    // Enable FIFO 0 and FIFO 1 new-message notifications
    last_status_ = HAL_FDCAN_ActivateNotification(
        hfdcan_,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
        0);
    if (last_status_ != HAL_OK) return false;
    last_status_ = HAL_FDCAN_Start(hfdcan_);
    return last_status_ == HAL_OK;
  }

  MYL_NOINLINE bool ImplSend(const CanFrame &frame) {
    FDCAN_TxHeaderTypeDef hdr{};
    hdr.Identifier          = frame.id;
    hdr.IdType              = frame.extended_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    hdr.TxFrameType         = frame.rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    hdr.DataLength          = DlcToFdCanDlc(frame.dlc);
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    hdr.FDFormat             = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;

    // HAL requires a non-const pointer for the data
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    last_status_ = HAL_FDCAN_AddMessageToTxFifoQ(
        hfdcan_, &hdr, const_cast<uint8_t *>(frame.data));
    return last_status_ == HAL_OK;
  }

  MYL_NOINLINE int ImplAddRxFilter(const CanFilter &filter,
                                    void (*callback)(const CanFrame &, void *),
                                    void *user_data) {
    if (filter_count_ >= kFdCanMaxFilters) return -1;

    uint8_t idx = filter_count_++;
    filters_[idx]    = filter;
    callbacks_[idx]  = callback;
    user_data_[idx]  = user_data;

    // Configure HAL filter element
    FDCAN_FilterTypeDef flt{};
    flt.IdType       = filter.extended_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    flt.FilterIndex  = idx;
    flt.FilterType   = FDCAN_FILTER_MASK;
    flt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    flt.FilterID1    = filter.id & (filter.extended_id ? 0x1FFFFFFFu : 0x7FFu);
    flt.FilterID2    = filter.mask & (filter.extended_id ? 0x1FFFFFFFu : 0x7FFu);

    last_status_ = HAL_FDCAN_ConfigFilter(hfdcan_, &flt);
    return (last_status_ == HAL_OK) ? static_cast<int>(idx) : -2;
  }

  MYL_NOINLINE bool ImplReadable() { return rx_wloc_ != rx_rloc_; }

  MYL_NOINLINE bool ImplGetFrame(CanFrame &frame) {
    if (!ImplReadable()) return false;
    frame = rx_queue_[rx_rloc_];
    rx_rloc_ = (rx_rloc_ + 1) & kQMask;
    return true;
  }

 private:
  // ---- Frame conversion helpers ----------------------------------------

  static uint32_t DlcToFdCanDlc(uint8_t dlc) {
    // Classical CAN: DLC 0-8 maps directly to FDCAN_DLC_BYTES_x
    static constexpr uint32_t kMap[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8,
    };
    return (dlc <= 8) ? kMap[dlc] : FDCAN_DLC_BYTES_8;
  }

  static uint8_t FdCanDlcToBytes(uint32_t fdcan_dlc) {
    // FDCAN_DLC_BYTES_x = (x << 16), so shift right by 16 gives byte count for 0-8
    uint8_t bytes = static_cast<uint8_t>(fdcan_dlc >> 16);
    return (bytes <= 8) ? bytes : 8;
  }

  /// Drain all pending messages from the given RX FIFO into the RX queue and callbacks.
  MYL_ISR_NOINLINE void DrainFifo(uint32_t fifo) {
    FDCAN_RxHeaderTypeDef hdr{};
    uint8_t raw[8]{};

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, fifo) > 0) {
      if (HAL_FDCAN_GetRxMessage(hfdcan_, fifo, &hdr, raw) != HAL_OK) break;

      CanFrame frm{};
      frm.id          = hdr.Identifier;
      frm.extended_id = (hdr.IdType == FDCAN_EXTENDED_ID);
      frm.rtr         = (hdr.RxFrameType == FDCAN_REMOTE_FRAME);
      frm.dlc         = FdCanDlcToBytes(hdr.DataLength);
      for (uint8_t i = 0; i < frm.dlc; ++i) frm.data[i] = raw[i];

      // Dispatch to matching callbacks
      bool dispatched = false;
      for (uint8_t i = 0; i < filter_count_; ++i) {
        const CanFilter &f = filters_[i];
        if (f.extended_id == frm.extended_id &&
            ((frm.id & f.mask) == (f.id & f.mask))) {
          if (callbacks_[i]) callbacks_[i](frm, user_data_[i]);
          dispatched = true;
        }
      }

      // Also push to polled queue (even if dispatched via callback)
      size_t next = (rx_wloc_ + 1) & kQMask;
      if (next != rx_rloc_) {
        rx_queue_[rx_wloc_] = frm;
        rx_wloc_ = next;
      }
      (void)dispatched;
    }
  }

  // ---- Fields ----------------------------------------------------------

  FDCAN_HandleTypeDef *hfdcan_;
  HAL_StatusTypeDef last_status_{HAL_OK};

  // RX callback filters
  uint8_t filter_count_{0};
  CanFilter filters_[kFdCanMaxFilters]{};
  void (*callbacks_[kFdCanMaxFilters])(const CanFrame &, void *){};
  void *user_data_[kFdCanMaxFilters]{};

  // Polled RX queue (power-of-2 ring buffer, ISR-safe on Cortex-M)
  CanFrame rx_queue_[RxQueueSize]{};
  volatile size_t rx_wloc_{0};
  volatile size_t rx_rloc_{0};
};

}  // namespace myl_utils
