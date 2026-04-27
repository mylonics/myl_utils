#pragma once

/**
 * @file peripheral.h
 * @brief CRTP base classes for SPI and I2C peripheral communication (zero virtual overhead)
 *
 * This module provides CRTP base classes for both synchronous and asynchronous
 * SPI/I2C communication. All dispatch is resolved at compile time — no vtables.
 *
 * Usage:
 * 1. Create a transport (e.g., ZephyrSpiTransport, Stm32SpiTransport)
 * 2. Wrap it in an SpiDevice/I2cDevice with per-device config (addr, CS, callback)
 * 3. Create Buffer objects and use Set()/Append() to populate TX data
 * 4. Build packets with factory methods (SpiPacket::WriteOnly, RegRead, etc.)
 * 5. Call SendSync(pkt) for blocking or SendAsync(pkt) for non-blocking transfers
 */

#include "buffer.h"
#include "config.h"
#include "gpio.h"
#include "myl_utils/noncopyable.h"
#include <cstdint>
#include <type_traits>

namespace myl_utils {

/**
 * @brief Specifies the type of SPI/I2C transfer to perform
 */
enum class PacketType : uint8_t {
  Read,          ///< Read-only operation
  Write,         ///< Write-only operation
  WriteAndRead,  ///< Simultaneous write and read (full-duplex for SPI)
  WriteThenRead  ///< Write first, then read (typical register read pattern)
};

/**
 * @brief Non-owning data buffer view for packet transfers
 * @note Maximum transfer length is 65535 bytes (uint16_t length field)
 */
struct Data {
  uint8_t *data;
  uint16_t length;  ///< Transfer length in bytes (max 65535)
  Data() : data{nullptr}, length{0} {}
  explicit Data(uint8_t *ptr) : data{ptr}, length{0} {}
  const uint8_t *cdata() const { return data; }
};

/**
 * @brief Owning data buffer that stores its own storage
 *
 * Inherits from Data so it can be used anywhere a Data& is expected.
 * The internal storage is automatically linked to the Data base pointer.
 *
 * @tparam Size Size of the internal buffer in bytes
 */
template <uint16_t Size>
struct Buffer : Data, NonCopyable<Buffer<Size>> {
  static constexpr uint16_t capacity = Size;
  uint8_t storage[Size]{};
  Buffer() : Data(storage) {}

  /// Reset buffer and write bytes, updating length automatically
  template <typename... Bytes>
  void Set(Bytes... bytes) {
    static_assert(sizeof...(bytes) <= Size, "Too many bytes for buffer");
    uint8_t tmp[] = {static_cast<uint8_t>(bytes)...};
    for (uint16_t i = 0; i < sizeof...(bytes); ++i) storage[i] = tmp[i];
    length = sizeof...(bytes);
  }

  /// Append bytes to existing data, updating length automatically
  /// @return Number of bytes actually appended (may be less than requested if buffer is full)
  template <typename... Bytes>
  uint16_t Append(Bytes... bytes) {
    static_assert(sizeof...(bytes) <= Size, "Too many bytes for buffer");
    uint8_t tmp[] = {static_cast<uint8_t>(bytes)...};
    uint16_t count = 0;
    for (uint16_t i = 0; i < sizeof...(bytes) && (length + i) < Size; ++i) {
      storage[length + i] = tmp[i];
      ++count;
    }
    length += count;
    return count;
  }

  /// Reset buffer length to zero
  void Clear() { length = 0; }
};

/**
 * @brief SPI clock polarity setting
 */
enum class SpiPolarity : uint8_t {
  Low = 0,   ///< CPOL=0, clock idle low
  High = 1   ///< CPOL=1, clock idle high
};

/**
 * @brief SPI clock phase setting
 */
enum class SpiPhase : uint8_t {
  Leading = 0,   ///< CPHA=0, data sampled on leading (first) clock edge
  Trailing = 1   ///< CPHA=1, data sampled on trailing (second) clock edge
};

/**
 * @brief Transfer mode for asynchronous peripheral operations
 *
 * Controls whether async transfers use interrupt-driven (IT) or DMA-based
 * hardware. Both modes use the same AsyncPacketSender queue/callback
 * infrastructure — only the underlying HAL/driver call differs.
 *
 * @note DMA mode requires that TX/RX buffers reside in DMA-accessible memory.
 * On STM32 F7/H7, this excludes DTCM/ITCM; ensure your SpiPacketBundle or
 * Buffer objects are allocated in a suitable RAM section.
 */
enum class TransferMode : uint8_t {
  Polling,    ///< Use blocking/polling HAL functions (no ISR)
  Interrupt,  ///< Use interrupt-based HAL functions (_IT)
  Dma         ///< Use DMA-based HAL functions (_DMA)
};

/**
 * @brief SPI transfer packet containing buffer pointers and transfer configuration
 *
 * Buffers are optional — set only tx_data for write-only, only rx_data for read-only,
 * or both for bidirectional transfers. Per-device fields (chip_select, callback,
 * polarity, phase, max_freq_hz) are typically set automatically by the SpiDevice wrapper.
 */
struct SpiPacket {
  // --- Pointer-aligned fields first (no internal padding holes) -------
  Data *tx_data{};               ///< TX buffer (nullptr if unused)
  Data *rx_data{};               ///< RX buffer (nullptr if unused)
  ChipSelectPin chip_select{};   ///< Optional chip select GPIO (type-erased, no virtual)
  void (*callback)(Data &){};    ///< Optional completion callback
  // --- 32-bit fields --------------------------------------------------
  uint32_t max_freq_hz{0};                  ///< 0 = use transport default; non-zero = cap SCLK for this transfer
  // --- Byte-sized fields grouped together at the end ------------------
  PacketType type{PacketType::Write};
  SpiPolarity polarity{SpiPolarity::Low};   ///< Clock polarity (set by SpiDevice wrapper)
  SpiPhase phase{SpiPhase::Leading};        ///< Clock phase (set by SpiDevice wrapper)

  /// Factory methods for creating typed packets (avoids fragile aggregate initialization)
  static SpiPacket WriteOnly(Data &tx) {
    SpiPacket p{}; p.tx_data = &tx; p.type = PacketType::Write; return p;
  }
  static SpiPacket ReadOnly(Data &rx) {
    SpiPacket p{}; p.rx_data = &rx; p.type = PacketType::Read; return p;
  }
  static SpiPacket FullDuplex(Data &tx, Data &rx) {
    SpiPacket p{}; p.tx_data = &tx; p.rx_data = &rx; p.type = PacketType::WriteAndRead; return p;
  }
  static SpiPacket RegRead(Data &tx, Data &rx) {
    SpiPacket p{}; p.tx_data = &tx; p.rx_data = &rx; p.type = PacketType::WriteThenRead; return p;
  }
};

// max_freq_hz fits in tail padding on 64-bit (6 pointers) but adds a word on 32-bit (7 pointers)
static_assert(sizeof(SpiPacket) == (sizeof(void *) >= 8 ? 6 : 7) * sizeof(void *),
              "SpiPacket has unexpected padding — check for stray #pragma pack");

/**
 * @brief I2C transfer packet containing buffer pointers and transfer configuration
 *
 * Buffers are optional — set only tx_data for write-only, only rx_data for read-only,
 * or both for bidirectional transfers. The addr field is typically set automatically
 * by the I2cDevice wrapper.
 */
struct I2cPacket {
  // --- Pointer-aligned fields first (no internal padding holes) -------
  Data *tx_data{};               ///< TX buffer (nullptr if unused)
  Data *rx_data{};               ///< RX buffer (nullptr if unused)
  void (*callback)(Data &){};    ///< Optional completion callback
  // --- Byte-sized fields grouped together at the end ------------------
  uint8_t addr{};                ///< I2C device address (set by I2cDevice wrapper)
  PacketType type{PacketType::Write};

  /// Factory methods for creating typed packets (avoids fragile aggregate initialization)
  /// @note No FullDuplex — I2C is half-duplex; use RegRead for register read patterns
  static I2cPacket WriteOnly(Data &tx) {
    I2cPacket p{}; p.tx_data = &tx; p.type = PacketType::Write; return p;
  }
  static I2cPacket ReadOnly(Data &rx) {
    I2cPacket p{}; p.rx_data = &rx; p.type = PacketType::Read; return p;
  }
  static I2cPacket RegRead(Data &tx, Data &rx) {
    I2cPacket p{}; p.tx_data = &tx; p.rx_data = &rx; p.type = PacketType::WriteThenRead; return p;
  }
};

static_assert(sizeof(I2cPacket) == 4 * sizeof(void *),
              "I2cPacket has unexpected padding — check for stray #pragma pack");

/**
 * @brief Type-safe SPI packet bundle with TX/RX buffers
 * @tparam Size Size of the TX and RX buffers in bytes
 *
 * Preferred over the PACKET_HELPER macros — provides proper scoping and type safety.
 * Usage: SpiPacketBundle<8> cmd; cmd.tx.Set(0x01, 0x02); device.SendSync(cmd.pkt);
 */
template <uint16_t Size>
struct SpiPacketBundle : NonCopyable<SpiPacketBundle<Size>> {
  Buffer<Size> tx;
  Buffer<Size> rx;
  SpiPacket pkt{&tx, &rx};

  SpiPacketBundle() = default;
};

/**
 * @brief Type-safe I2C packet bundle with TX/RX buffers
 * @tparam Size Size of the TX and RX buffers in bytes
 *
 * Preferred over the PACKET_HELPER macros — provides proper scoping and type safety.
 * Usage: I2cPacketBundle<8> cmd; cmd.tx.Set(0x01, 0x02); device.SendSync(cmd.pkt);
 */
template <uint16_t Size>
struct I2cPacketBundle : NonCopyable<I2cPacketBundle<Size>> {
  Buffer<Size> tx;
  Buffer<Size> rx;
  I2cPacket pkt{&tx, &rx};

  I2cPacketBundle() = default;
};

/**
 * @brief CRTP base class for asynchronous packet-based transfers
 *
 * Provides queuing functionality for non-blocking transfers. When a transfer
 * completes, call TxRxCmpltCb() from the interrupt handler to process the
 * next queued packet.
 *
 * Derived classes must implement (as non-virtual methods):
 *   bool AsyncReadWritePacket(DataPacket &pkt)
 *   bool AsyncWritePacket(DataPacket &pkt)
 *   bool AsyncReadPacket(DataPacket &pkt)
 *   void ChipSelect(DataPacket &pkt, bool enable)
 *
 * @note ISR safety assumes ARM Cortex-M memory model (single-core, strongly-ordered
 *       peripheral writes). The volatile qualifier on busy_ and the circular buffer
 *       indices is sufficient on this architecture without explicit memory barriers.
 *
 * @tparam Derived The concrete implementation class (CRTP parameter)
 * @tparam DataPacket Either SpiPacket or I2cPacket
 * @tparam QueueSize Maximum number of packets that can be queued (must be power of 2, default: 32)
 */
template <class Derived, class DataPacket, uint8_t QueueSize = 32>
class AsyncPacketSender : NonCopyable<AsyncPacketSender<Derived, DataPacket, QueueSize>> {
 protected:
  /// @note Access from both thread and ISR context — volatile is sufficient on ARM Cortex-M
  volatile bool busy_{};

 public:
  AsyncPacketSender()
      : pkt_producer_(pkt_queue_.MakeProducer()),
        pkt_consumer_(pkt_queue_.MakeConsumer()) {}

  /**
   * @brief Check whether an async transfer is currently in progress
   * @return true if the async engine is busy (transfer in-flight or queued)
   */
  bool IsAsyncBusy() const { return busy_; }

  /**
   * @brief Queue a packet for asynchronous transfer
   * @param pkt Packet to transfer (must outlive the async operation)
   * @return true if packet was accepted, false if queue is full
   */
  [[nodiscard]] MYL_NOINLINE bool SendAsync(DataPacket &pkt) { return QueuePacket(pkt); }

  /**
   * @brief Call from interrupt handler when transfer completes
   *
   * Handles WriteThenRead sequencing and invokes completion callbacks.
   * Automatically processes next queued packet if available.
   *
   * If the read phase of a WriteThenRead fails to start, falls through
   * to cleanup (ChipSelect deassert, callback, next packet).
   */
  MYL_NOINLINE void TxRxCmpltCb() {
    if (current_command_->type == PacketType::WriteThenRead && write_and_read_pkt_) {
      write_and_read_pkt_ = false;
      if (derived().AsyncReadPacket(*current_command_)) return;
      // Read failed to start — fall through to cleanup
    }

    derived().ChipSelect(*current_command_, false);
    if (current_command_->callback) {
      if (current_command_->rx_data) {
        current_command_->callback(*current_command_->rx_data);
      } else if (current_command_->tx_data) {
        current_command_->callback(*current_command_->tx_data);
      }
    }

    DataPacket *next;
    if (pkt_consumer_.TryGet(next)) {
      AsyncSendPacket(*next);
    } else {
      busy_ = false;
    }
  }

 private:
  CircularBuffer<DataPacket *, QueueSize> pkt_queue_;
  BufferProducer<DataPacket *> pkt_producer_;
  BufferConsumer<DataPacket *> pkt_consumer_;
  bool write_and_read_pkt_{};
  DataPacket *current_command_{nullptr};

  Derived &derived() { return static_cast<Derived &>(*this); }

  MYL_NOINLINE bool QueuePacket(DataPacket &pkt) {
    if (busy_) {
      return pkt_producer_.TryPut(&pkt);
    } else {
      return AsyncSendPacket(pkt);
    }
  }

  MYL_NOINLINE bool AsyncSendPacket(DataPacket &pkt) {
    busy_ = true;
    write_and_read_pkt_ = false;
    current_command_ = &pkt;
    derived().ChipSelect(pkt, true);
    bool ok = false;
    switch (pkt.type) {
      case PacketType::Read:
        ok = derived().AsyncReadPacket(pkt);
        break;
      case PacketType::WriteAndRead:
        ok = derived().AsyncReadWritePacket(pkt);
        break;
      case PacketType::WriteThenRead:
        write_and_read_pkt_ = true;
        ok = derived().AsyncWritePacket(pkt);
        break;
      case PacketType::Write:
      default:
        ok = derived().AsyncWritePacket(pkt);
        break;
    }
    if (!ok) {
      derived().ChipSelect(pkt, false);
      busy_ = false;
    }
    return ok;
  }
};

/**
 * @brief CRTP base class for synchronous packet-based transfers
 *
 * Provides blocking transfer functionality. Each call to SendSync()
 * completes the entire transfer before returning.
 *
 * Derived classes must implement (as non-virtual methods):
 *   bool SyncReadWritePacket(DataPacket &pkt)
 *   bool SyncWritePacket(DataPacket &pkt)
 *   bool SyncReadPacket(DataPacket &pkt)
 *   void ChipSelect(DataPacket &pkt, bool enable)
 *
 * @tparam Derived The concrete implementation class (CRTP parameter)
 * @tparam DataPacket Either SpiPacket or I2cPacket
 */
template <class Derived, class DataPacket>
class SyncPacketSender : NonCopyable<SyncPacketSender<Derived, DataPacket>> {
 public:
  /**
   * @brief Execute a packet transfer (blocking)
   * @param pkt Packet to transfer
   * @return true if the transfer succeeded, false on hardware error
   */
  [[nodiscard]] MYL_NOINLINE bool SendSync(DataPacket &pkt) {
    return SyncSendPacket(pkt);
  }

 private:
  Derived &derived() { return static_cast<Derived &>(*this); }

  /// SFINAE helper: detect if Derived has IsAsyncBusy()
  template <typename D, typename = void>
  struct HasAsyncBusy : std::false_type {};
  template <typename D>
  struct HasAsyncBusy<D, std::void_t<decltype(std::declval<D>().IsAsyncBusy())>> : std::true_type {};

  MYL_NOINLINE bool SyncSendPacket(DataPacket &pkt) {
    // If the derived type also inherits AsyncPacketSender, refuse to start
    // a blocking transfer while an async transfer is in-flight.
    if constexpr (HasAsyncBusy<Derived>::value) {
      if (derived().IsAsyncBusy()) return false;
    }
    bool ok = true;
    derived().ChipSelect(pkt, true);
    switch (pkt.type) {
      case PacketType::Read:
        ok = derived().SyncReadPacket(pkt);
        break;
      case PacketType::WriteAndRead:
        ok = derived().SyncReadWritePacket(pkt);
        break;
      case PacketType::WriteThenRead:
        ok = derived().SyncWritePacket(pkt);
        if (ok) ok = derived().SyncReadPacket(pkt);
        break;
      case PacketType::Write:
      default:
        ok = derived().SyncWritePacket(pkt);
        break;
    }
    derived().ChipSelect(pkt, false);
    return ok;
  }
};

/// Synchronous SPI CRTP base — usage: class MySpi : public SyncSpi<MySpi> { ... };
template <typename Derived>
using SyncSpi = SyncPacketSender<Derived, SpiPacket>;

/// Synchronous I2C CRTP base — usage: class MyI2c : public SyncI2c<MyI2c> { ... };
template <typename Derived>
using SyncI2c = SyncPacketSender<Derived, I2cPacket>;

/// Asynchronous SPI CRTP base — usage: class MySpi : public AsyncSpi<MySpi> { ... };
template <typename Derived, uint8_t QueueSize = 32>
using AsyncSpi = AsyncPacketSender<Derived, SpiPacket, QueueSize>;

/// Asynchronous I2C CRTP base — usage: class MyI2c : public AsyncI2c<MyI2c> { ... };
template <typename Derived, uint8_t QueueSize = 32>
using AsyncI2c = AsyncPacketSender<Derived, I2cPacket, QueueSize>;

/**
 * @brief SPI peripheral device wrapper
 *
 * Stores a reference to the SPI transport and per-device configuration
 * (chip select, callback, polarity, phase). Automatically stamps each
 * packet with device-specific settings before forwarding to the transport.
 *
 * Exposes SendSync() and/or SendAsync() depending on what the transport
 * supports. A transport inheriting SyncSpi provides SendSync(); a transport
 * inheriting AsyncSpi provides SendAsync(); a transport inheriting both
 * provides both methods through a single device object.
 *
 * Usage:
 * @code
 * // Sync-only transport:
 * SpiDevice<ZephyrSpiTransport> sensor(spi, ChipSelectPin(cs_gpio));
 * sensor.SendSync(pkt);
 *
 * // Unified transport (sync + async):
 * SpiDevice<Stm32SpiTransport<>> sensor(spi, ChipSelectPin(cs_gpio), my_callback);
 * sensor.SendSync(pkt);       // blocking register read during init
 * sensor.SendAsync(bulk.pkt); // non-blocking DMA read at runtime
 * @endcode
 *
 * @tparam Transport SPI transport type (class inheriting SyncSpi<>, AsyncSpi<>, or both)
 */
template <class Transport>
class SpiDevice {
  Transport &transport_;
  ChipSelectPin chip_select_{};
  void (*callback_)(Data &){};
  SpiPolarity polarity_{SpiPolarity::Low};
  SpiPhase phase_{SpiPhase::Leading};
  uint32_t max_freq_hz_{0};

  MYL_NOINLINE void StampPacket(SpiPacket &pkt) {
    pkt.chip_select = chip_select_;
    pkt.polarity = polarity_;
    pkt.phase = phase_;
    if (pkt.max_freq_hz == 0) pkt.max_freq_hz = max_freq_hz_;
    if (!pkt.callback) pkt.callback = callback_;
  }

 public:
  /// Construct without chip select
  explicit SpiDevice(Transport &transport, void (*cb)(Data &) = nullptr,
                     SpiPolarity pol = SpiPolarity::Low, SpiPhase pha = SpiPhase::Leading,
                     uint32_t max_freq_hz = 0)
      : transport_(transport), callback_(cb), polarity_(pol), phase_(pha),
        max_freq_hz_(max_freq_hz) {}

  /// Construct with chip select
  /// @param cs ChipSelectPin wrapping any GPIO with Set(bool).
  ///           Defaults to active-low — see ChipSelectPin for polarity control.
  SpiDevice(Transport &transport, ChipSelectPin cs, void (*cb)(Data &) = nullptr,
            SpiPolarity pol = SpiPolarity::Low, SpiPhase pha = SpiPhase::Leading,
            uint32_t max_freq_hz = 0)
      : transport_(transport), chip_select_(cs), callback_(cb), polarity_(pol), phase_(pha),
        max_freq_hz_(max_freq_hz) {}

  /// Set SPI clock polarity and phase
  void SetSpiMode(SpiPolarity pol, SpiPhase pha) {
    polarity_ = pol;
    phase_ = pha;
  }

  /// Set the maximum SCLK frequency for all transfers on this device (0 = no limit)
  void SetMaxFrequency(uint32_t hz) { max_freq_hz_ = hz; }

  /// Execute a blocking transfer (requires transport with SyncSpi base)
  [[nodiscard]] MYL_NOINLINE bool SendSync(SpiPacket &pkt) {
    StampPacket(pkt);
    return transport_.SendSync(pkt);
  }

  /// Queue an async transfer (requires transport with AsyncSpi base). Packet must outlive the transfer.
  [[nodiscard]] MYL_NOINLINE bool SendAsync(SpiPacket &pkt) {
    StampPacket(pkt);
    return transport_.SendAsync(pkt);
  }

  /// Access the underlying transport (e.g., to check last_status() / last_error(), call TxRxCmpltCb())
  Transport &transport() { return transport_; }
  const Transport &transport() const { return transport_; }
};

/**
 * @brief I2C peripheral device wrapper
 *
 * Stores a reference to the I2C transport and per-device configuration
 * (address, callback). Automatically stamps each packet with device-specific
 * settings before forwarding to the transport.
 *
 * Exposes SendSync() and/or SendAsync() depending on what the transport
 * supports. A transport inheriting SyncI2c provides SendSync(); a transport
 * inheriting AsyncI2c provides SendAsync(); a transport inheriting both
 * provides both methods through a single device object.
 *
 * Usage:
 * @code
 * // Sync-only:
 * I2cDevice<ZephyrI2cTransport> eeprom(i2c, 0x50);
 * eeprom.SendSync(pkt);
 *
 * // Unified transport:
 * I2cDevice<Stm32I2cTransport<>> sensor(i2c, 0x68, my_callback);
 * sensor.SendSync(pkt);        // blocking register read
 * sensor.SendAsync(bulk.pkt);  // non-blocking DMA transfer
 * @endcode
 *
 * @tparam Transport I2C transport type (class inheriting SyncI2c<>, AsyncI2c<>, or both)
 */
template <class Transport>
class I2cDevice {
  Transport &transport_;
  uint8_t addr_{};
  void (*callback_)(Data &){};

  MYL_NOINLINE void StampPacket(I2cPacket &pkt) {
    pkt.addr = addr_;
    if (!pkt.callback) pkt.callback = callback_;
  }

 public:
  I2cDevice(Transport &transport, uint8_t addr, void (*cb)(Data &) = nullptr)
      : transport_(transport), addr_(addr), callback_(cb) {}

  /// Execute a blocking transfer (requires transport with SyncI2c base)
  [[nodiscard]] MYL_NOINLINE bool SendSync(I2cPacket &pkt) {
    StampPacket(pkt);
    return transport_.SendSync(pkt);
  }

  /// Queue an async transfer (requires transport with AsyncI2c base). Packet must outlive the transfer.
  [[nodiscard]] MYL_NOINLINE bool SendAsync(I2cPacket &pkt) {
    StampPacket(pkt);
    return transport_.SendAsync(pkt);
  }

  /// Access the underlying transport (e.g., to check last_status() / last_error(), call TxRxCmpltCb())
  Transport &transport() { return transport_; }
  const Transport &transport() const { return transport_; }
};

}  // namespace myl_utils
