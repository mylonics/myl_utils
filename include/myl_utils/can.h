#pragma once

/**
 * @file can.h
 * @brief CRTP base class for CAN transports (zero virtual overhead)
 *
 * Provides:
 *   CanFrame   — standard CAN frame (classical CAN, up to 8 bytes DLC)
 *   CanFilter  — ID + mask acceptance filter
 *   CanBase<Derived> — CRTP base dispatching to concrete transport Impl* methods
 *
 * Concrete transports:
 *   ZephyrCanTransport   — include <myl_utils/zephyr/can.h>
 *   Stm32FdCanTransport  — include <myl_utils/stm32/can.h>
 *
 * Usage:
 * @code
 * // Zephyr example
 * #include <myl_utils/zephyr/can.h>
 * ZephyrCanTransport<> can(DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)));
 * can.Start();
 *
 * CanFrame tx;
 * tx.id = 0x123;
 * tx.dlc = 3;
 * tx.data[0] = 0xDE; tx.data[1] = 0xAD; tx.data[2] = 0xBE;
 * can.Send(tx);
 *
 * CanFilter f{.id = 0x200, .mask = 0x7FF, .extended_id = false};
 * can.AddRxFilter(f, [](const CanFrame &frm){ /* handle */ });
 * @endcode
 *
 * Concrete transport requirements (non-virtual methods in Derived):
 *   bool ImplStart()
 *   bool ImplSend(const CanFrame &)
 *   bool ImplAddRxFilter(const CanFilter &, void (*callback)(const CanFrame &, void *), void *user_data)
 *   bool ImplReadable()
 *   bool ImplGetFrame(CanFrame &)
 */

#include "config.h"
#include "myl_utils/noncopyable.h"
#include <cstdint>
#include <cstring>

namespace myl_utils {

// ---------------------------------------------------------------------------
// CAN frame type
// ---------------------------------------------------------------------------

/// Maximum DLC for classical CAN (8 bytes). FDCAN FD data-phase not yet supported.
static constexpr uint8_t kCanMaxDlc = 8;

/**
 * @brief Standard CAN frame (classical CAN, up to 8 data bytes).
 *
 * For FDCAN FD-mode payloads (> 8 bytes) a separate CanFdFrame type will be
 * added in a future revision.
 */
struct CanFrame {
  uint32_t id{0};           ///< CAN ID (11-bit standard or 29-bit extended)
  uint8_t  dlc{0};          ///< Data length code (0–8)
  uint8_t  data[kCanMaxDlc]{};
  bool     extended_id{false}; ///< true = 29-bit extended ID
  bool     rtr{false};         ///< true = Remote Transmission Request

  CanFrame() = default;

  /// Construct a data frame in one shot.
  static CanFrame Make(uint32_t id, bool extended, const uint8_t *payload, uint8_t len) {
    CanFrame f{};
    f.id = id;
    f.extended_id = extended;
    f.dlc = (len > kCanMaxDlc) ? kCanMaxDlc : len;
    memcpy(f.data, payload, f.dlc);
    return f;
  }
};

// ---------------------------------------------------------------------------
// CAN acceptance filter
// ---------------------------------------------------------------------------

/**
 * @brief CAN acceptance filter (ID + bitmask).
 *
 * A received frame passes the filter when:
 *   (frame.id & mask) == (id & mask)
 */
struct CanFilter {
  uint32_t id{0};           ///< Target CAN ID (standard or extended)
  uint32_t mask{0x7FFu};    ///< Bitmask — 0x7FF = exact match for 11-bit, 0x1FFFFFFF for 29-bit
  bool     extended_id{false}; ///< Must match frame.extended_id
};

// ---------------------------------------------------------------------------
// CanBase<Derived> — CRTP base, zero virtual overhead
// ---------------------------------------------------------------------------

/**
 * @brief CRTP base for all CAN transports.
 *
 * All public methods dispatch to Derived::Impl*() at compile time.
 *
 * @tparam Derived Concrete transport class (ZephyrCanTransport, Stm32FdCanTransport, …)
 */
template <class Derived>
class CanBase {
 public:
  /// Initialize and start the CAN peripheral (set bitrate, enable, enter normal mode).
  MYL_NOINLINE bool Start() { return derived().ImplStart(); }

  /// Transmit a CAN frame. Returns true if the frame was accepted by the hardware.
  /// @note In async transports (Zephyr) the frame may not be sent yet when this returns.
  MYL_NOINLINE bool Send(const CanFrame &frame) { return derived().ImplSend(frame); }

  /**
   * @brief Add an RX acceptance filter and register a callback.
   *
   * The callback is invoked from an ISR or system work-queue context (platform-dependent).
   * @param filter     Acceptance filter (id, mask, extended_id flag)
   * @param callback   Function called for each matching received frame
   * @param user_data  Opaque pointer passed to the callback
   * @return Filter handle (≥ 0) on success, or negative error code on failure.
   */
  MYL_NOINLINE int AddRxFilter(const CanFilter &filter,
                                void (*callback)(const CanFrame &, void *),
                                void *user_data = nullptr) {
    return derived().ImplAddRxFilter(filter, callback, user_data);
  }

  /// Return true if at least one received frame is available in the RX queue.
  /// @note Valid only when using the polled GetFrame() API (no callback registered).
  MYL_NOINLINE bool Readable() { return derived().ImplReadable(); }

  /**
   * @brief Read one received frame (polled, non-blocking).
   *
   * @param frame  Output frame
   * @return true if a frame was read, false if the RX queue was empty.
   */
  MYL_NOINLINE bool GetFrame(CanFrame &frame) { return derived().ImplGetFrame(frame); }

 private:
  Derived &derived() { return *static_cast<Derived *>(this); }
  const Derived &derived() const { return *static_cast<const Derived *>(this); }
};

}  // namespace myl_utils
