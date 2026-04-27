#pragma once

/**
 * @file zephyr/can.h
 * @brief Zephyr CAN transport for CanBase<Derived> CRTP
 *
 * Wraps the Zephyr CAN API (can_send / can_add_rx_filter) for use with
 * CanBase<Derived>. Requires CONFIG_MYL_UTILS_CAN (selects CAN).
 *
 * Transmit: can_send() — non-blocking with an optional TX-done callback.
 * Receive:  can_add_rx_filter() — callback invoked from the CAN driver's
 *           work-queue or ISR for each accepted frame.
 *           Received frames are also stored in an internal ring buffer so
 *           callers can use the polled GetFrame() / Readable() API.
 *
 * Usage:
 * @code
 * #include <myl_utils/zephyr/can.h>
 *
 * myl_utils::ZephyrCanTransport<> can(DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)));
 * can.Start();
 *
 * // Add a filter with callback
 * myl_utils::CanFilter f{.id = 0x123, .mask = 0x7FF};
 * can.AddRxFilter(f, [](const myl_utils::CanFrame &frm, void *) {
 *   // handle frame — called from CAN RX work-queue
 * });
 *
 * // Send a frame
 * myl_utils::CanFrame tx;
 * tx.id = 0x200; tx.dlc = 2; tx.data[0] = 0xAB; tx.data[1] = 0xCD;
 * can.Send(tx);
 *
 * // Polled RX
 * myl_utils::CanFrame rx;
 * if (can.Readable()) { can.GetFrame(rx); }
 * @endcode
 */

#include <myl_utils/can.h>
#include <myl_utils/noncopyable.h>

#ifndef CONFIG_MYL_UTILS_CAN
#error "Enable CONFIG_MYL_UTILS_CAN in Kconfig (prj.conf)"
#endif

#include <zephyr/drivers/can.h>
#include <zephyr/sys/ring_buffer.h>

namespace myl_utils {

/// Maximum number of RX filters per ZephyrCanTransport instance.
static constexpr uint8_t kZephyrCanMaxFilters = 8;

/**
 * @brief Zephyr CAN transport.
 *
 * @tparam RxQueueFrames  Depth of the polled RX frame ring buffer (number of CanFrame slots)
 */
template <size_t RxQueueFrames = 16>
class ZephyrCanTransport
    : public CanBase<ZephyrCanTransport<RxQueueFrames>>,
      NonCopyable<ZephyrCanTransport<RxQueueFrames>> {
  friend class CanBase<ZephyrCanTransport<RxQueueFrames>>;

  static_assert((RxQueueFrames & (RxQueueFrames - 1)) == 0,
                "RxQueueFrames must be a power of 2");

  static constexpr size_t kQMask = RxQueueFrames - 1;

 public:
  explicit ZephyrCanTransport(const struct device *dev) : dev_(dev) {}

 protected:
  // ---- CanBase<Derived> interface --------------------------------------

  MYL_NOINLINE bool ImplStart() {
    return can_start(dev_) == 0;
  }

  MYL_NOINLINE bool ImplSend(const CanFrame &frame) {
    struct can_frame zfrm{};
    zfrm.id    = frame.id;
    zfrm.dlc   = frame.dlc;
    zfrm.flags = 0;
    if (frame.extended_id) zfrm.flags |= CAN_FRAME_IDE;
    if (frame.rtr)         zfrm.flags |= CAN_FRAME_RTR;
    for (uint8_t i = 0; i < frame.dlc && i < CAN_MAX_DLEN; ++i) {
      zfrm.data[i] = frame.data[i];
    }
    return can_send(dev_, &zfrm, K_NO_WAIT, nullptr, nullptr) == 0;
  }

  MYL_NOINLINE int ImplAddRxFilter(const CanFilter &filter,
                                    void (*callback)(const CanFrame &, void *),
                                    void *user_data) {
    if (filter_count_ >= kZephyrCanMaxFilters) return -ENOMEM;

    struct can_filter zflt{};
    zflt.id   = filter.id;
    zflt.mask = filter.mask;
    zflt.flags = 0;
    if (filter.extended_id) zflt.flags |= CAN_FILTER_IDE;

    // All filters route to the same shared Zephyr callback; we demux by ID.
    int handle = can_add_rx_filter(dev_, ZephyrRxCallback, this, &zflt);
    if (handle < 0) return handle;

    uint8_t idx          = filter_count_++;
    filters_[idx]        = filter;
    callbacks_[idx]      = callback;
    user_data_ptr_[idx]  = user_data;
    zephyr_handles_[idx] = handle;
    return idx;
  }

  MYL_NOINLINE bool ImplReadable() {
    k_spinlock_key_t key = k_spin_lock(&lock_);
    bool result = rx_wloc_ != rx_rloc_;
    k_spin_unlock(&lock_, key);
    return result;
  }

  MYL_NOINLINE bool ImplGetFrame(CanFrame &frame) {
    k_spinlock_key_t key = k_spin_lock(&lock_);
    if (rx_wloc_ == rx_rloc_) {
      k_spin_unlock(&lock_, key);
      return false;
    }
    frame = rx_queue_[rx_rloc_];
    rx_rloc_ = (rx_rloc_ + 1) & kQMask;
    k_spin_unlock(&lock_, key);
    return true;
  }

 private:
  // ---- Zephyr RX callback (called per-filter from CAN driver work-queue) ---

  static void ZephyrRxCallback(const struct device *dev,
                                struct can_frame *zfrm,
                                void *user_data) {
    auto *self = static_cast<ZephyrCanTransport *>(user_data);

    // Convert Zephyr frame to myl_utils CanFrame
    CanFrame frm{};
    frm.id          = zfrm->id;
    frm.dlc         = zfrm->dlc;
    frm.extended_id = (zfrm->flags & CAN_FRAME_IDE) != 0;
    frm.rtr         = (zfrm->flags & CAN_FRAME_RTR) != 0;
    for (uint8_t i = 0; i < frm.dlc && i < CAN_MAX_DLEN; ++i) {
      frm.data[i] = zfrm->data[i];
    }

    // Dispatch to user callbacks for matching filters
    for (uint8_t i = 0; i < self->filter_count_; ++i) {
      const CanFilter &f = self->filters_[i];
      if (f.extended_id == frm.extended_id &&
          ((frm.id & f.mask) == (f.id & f.mask))) {
        if (self->callbacks_[i]) self->callbacks_[i](frm, self->user_data_ptr_[i]);
      }
    }

    // Also enqueue into the polled RX ring buffer
    {
      k_spinlock_key_t key = k_spin_lock(&self->lock_);
      size_t next = (self->rx_wloc_ + 1) & kQMask;
      if (next != self->rx_rloc_) {
        self->rx_queue_[self->rx_wloc_] = frm;
        self->rx_wloc_ = next;
      }
      k_spin_unlock(&self->lock_, key);
    }
  }

  // ---- Fields ----------------------------------------------------------

  const struct device *dev_;

  // Filter + callback registry
  uint8_t filter_count_{0};
  CanFilter filters_[kZephyrCanMaxFilters]{};
  void (*callbacks_[kZephyrCanMaxFilters])(const CanFrame &, void *){};
  void *user_data_ptr_[kZephyrCanMaxFilters]{};
  int zephyr_handles_[kZephyrCanMaxFilters]{};

  // Polled RX queue — protected by lock_ since ZephyrRxCallback runs from the
  // CAN work-queue thread and may interleave with app reads.
  mutable struct k_spinlock lock_;
  CanFrame rx_queue_[RxQueueFrames]{};
  volatile size_t rx_wloc_{0};
  volatile size_t rx_rloc_{0};
};

}  // namespace myl_utils
