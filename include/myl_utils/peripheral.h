#pragma once

/**
 * @file peripheral.h
 * @brief CRTP base classes for SPI and I2C peripheral communication (zero virtual overhead)
 *
 * This module provides CRTP base classes for both synchronous and asynchronous
 * SPI/I2C communication. All dispatch is resolved at compile time — no vtables.
 *
 * Usage:
 * 1. Create a transport (e.g., ZephyrSpiDevice, Stm32SpiDevice)
 * 2. Wrap it in an SpiDevice/I2cDevice with per-device config (addr, CS, callback)
 * 3. Create Buffer objects and use Set()/Append() to populate TX data
 * 4. Use convenience methods (Write, Read, WriteThenRead) or ProcessCommand()
 */

#include "buffer.h"
#include "gpio.h"
#include "myl_utils/noncopyable.h"
#include <cstdint>

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
struct Buffer : Data {
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
  Interrupt,  ///< Use interrupt-based HAL functions (_IT)
  Dma         ///< Use DMA-based HAL functions (_DMA)
};

/**
 * @brief SPI transfer packet containing buffer pointers and transfer configuration
 *
 * Buffers are optional — set only tx_data for write-only, only rx_data for read-only,
 * or both for bidirectional transfers. Per-device fields (chip_select, callback,
 * polarity, phase) are typically set automatically by the SpiDevice wrapper.
 */
struct SpiPacket {
  Data *tx_data{};               ///< TX buffer (nullptr if unused)
  Data *rx_data{};               ///< RX buffer (nullptr if unused)
  PacketType type{PacketType::Write};
  ChipSelectPin chip_select{};   ///< Optional chip select GPIO (type-erased, no virtual)
  void (*callback)(Data &){};    ///< Optional completion callback
  SpiPolarity polarity{SpiPolarity::Low};   ///< Clock polarity (set by SpiDevice wrapper)
  SpiPhase phase{SpiPhase::Leading};        ///< Clock phase (set by SpiDevice wrapper)

  /// Factory methods for creating typed packets (avoids fragile aggregate initialization)
  static SpiPacket WriteOnly(Data &tx) { return {&tx, nullptr, PacketType::Write}; }
  static SpiPacket ReadOnly(Data &rx) { return {nullptr, &rx, PacketType::Read}; }
  static SpiPacket FullDuplex(Data &tx, Data &rx) { return {&tx, &rx, PacketType::WriteAndRead}; }
  static SpiPacket RegRead(Data &tx, Data &rx) { return {&tx, &rx, PacketType::WriteThenRead}; }
};

/**
 * @brief I2C transfer packet containing buffer pointers and transfer configuration
 *
 * Buffers are optional — set only tx_data for write-only, only rx_data for read-only,
 * or both for bidirectional transfers. The addr field is typically set automatically
 * by the I2cDevice wrapper.
 */
struct I2cPacket {
  Data *tx_data{};               ///< TX buffer (nullptr if unused)
  Data *rx_data{};               ///< RX buffer (nullptr if unused)
  uint8_t addr{};                ///< I2C device address (set by I2cDevice wrapper)
  PacketType type{PacketType::Write};
  void (*callback)(Data &){};    ///< Optional completion callback

  /// Factory methods for creating typed packets (avoids fragile aggregate initialization)
  /// @note No FullDuplex — I2C is half-duplex; use RegRead for register read patterns
  static I2cPacket WriteOnly(Data &tx) { return {&tx, nullptr, 0, PacketType::Write}; }
  static I2cPacket ReadOnly(Data &rx) { return {nullptr, &rx, 0, PacketType::Read}; }
  static I2cPacket RegRead(Data &tx, Data &rx) { return {&tx, &rx, 0, PacketType::WriteThenRead}; }
};

/**
 * @brief Type-safe SPI packet bundle with TX/RX buffers
 * @tparam Size Size of the TX and RX buffers in bytes
 *
 * Preferred over the PACKET_HELPER macros — provides proper scoping and type safety.
 * Usage: SpiPacketBundle<8> cmd; cmd.tx.Set(0x01, 0x02); device.ProcessCommand(cmd.pkt);
 */
template <uint16_t Size>
struct SpiPacketBundle {
  Buffer<Size> tx;
  Buffer<Size> rx;
  SpiPacket pkt{&tx, &rx};
};

/**
 * @brief Type-safe I2C packet bundle with TX/RX buffers
 * @tparam Size Size of the TX and RX buffers in bytes
 *
 * Preferred over the PACKET_HELPER macros — provides proper scoping and type safety.
 * Usage: I2cPacketBundle<8> cmd; cmd.tx.Set(0x01, 0x02); device.ProcessCommand(cmd.pkt);
 */
template <uint16_t Size>
struct I2cPacketBundle {
  Buffer<Size> tx;
  Buffer<Size> rx;
  I2cPacket pkt{&tx, &rx};
};

// Legacy macros — prefer SpiPacketBundle<Size> / I2cPacketBundle<Size> instead
#define PACKET_HELPER(name, size) \
  myl_utils::Buffer<size> name##_tx_;        \
  myl_utils::Buffer<size> name##_rx_;

#define I2C_PACKET_HELPER(name, size) \
  PACKET_HELPER(name, size)           \
  myl_utils::I2cPacket name{&name##_tx_, &name##_rx_};

#define SPI_PACKET_HELPER(name, size) \
  PACKET_HELPER(name, size)           \
  myl_utils::SpiPacket name{&name##_tx_, &name##_rx_};

/**
 * @brief CRTP base class for asynchronous packet-based transfers
 *
 * Provides queuing functionality for non-blocking transfers. When a transfer
 * completes, call TxRxCmpltCb() from the interrupt handler to process the
 * next queued packet.
 *
 * Derived classes must implement (as non-virtual methods):
 *   bool ReadWritePacket(DataPacket &pkt)
 *   bool WritePacket(DataPacket &pkt)
 *   bool ReadPacket(DataPacket &pkt)
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
  static constexpr bool is_sync = false;
  /**
   * @brief Queue a packet for transfer
   * @param pkt Packet to transfer
   * @return true if packet was accepted, false if queue is full
   */
  [[nodiscard]] bool ProcessCommand(DataPacket &pkt) { return QueuePacket(pkt); }

  /**
   * @brief Call from interrupt handler when transfer completes
   *
   * Handles WriteThenRead sequencing and invokes completion callbacks.
   * Automatically processes next queued packet if available.
   *
   * If the read phase of a WriteThenRead fails to start, falls through
   * to cleanup (ChipSelect deassert, callback, next packet).
   */
  void TxRxCmpltCb() {
    if (current_command_->type == PacketType::WriteThenRead && write_and_read_pkt_) {
      write_and_read_pkt_ = false;
      if (derived().ReadPacket(*current_command_)) return;
      // Read failed to start — fall through to cleanup
    }

    derived().ChipSelect(*current_command_, false);
    if (current_command_->callback && current_command_->rx_data) {
      current_command_->callback(*current_command_->rx_data);
    }

    if (pkt_queue_.Readable()) {
      DataPacket &temp_pkt_holder = *pkt_queue_.Get();
      SendPacket(temp_pkt_holder);
    } else {
      busy_ = false;
    }
  }

 private:
  CircularBuffer<DataPacket *, QueueSize> pkt_queue_;
  bool write_and_read_pkt_{};
  DataPacket *current_command_{nullptr};

  Derived &derived() { return static_cast<Derived &>(*this); }

  bool QueuePacket(DataPacket &pkt) {
    if (busy_) {
      if (!pkt_queue_.Writable()) return false;
      pkt_queue_.Put(&pkt);
      return true;
    } else {
      return SendPacket(pkt);
    }
  }

  bool SendPacket(DataPacket &pkt) {
    busy_ = true;
    write_and_read_pkt_ = false;
    current_command_ = &pkt;
    derived().ChipSelect(pkt, true);
    bool ok = false;
    switch (pkt.type) {
      case PacketType::Read:
        ok = derived().ReadPacket(pkt);
        break;
      case PacketType::WriteAndRead:
        ok = derived().ReadWritePacket(pkt);
        break;
      case PacketType::WriteThenRead:
        write_and_read_pkt_ = true;
        ok = derived().WritePacket(pkt);
        break;
      case PacketType::Write:
      default:
        ok = derived().WritePacket(pkt);
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
 * Provides blocking transfer functionality. Each call to ProcessCommand()
 * completes the entire transfer before returning.
 *
 * Derived classes must implement (as non-virtual methods):
 *   bool ReadWritePacket(DataPacket &pkt)
 *   bool WritePacket(DataPacket &pkt)
 *   bool ReadPacket(DataPacket &pkt)
 *   void ChipSelect(DataPacket &pkt, bool enable)
 *
 * @tparam Derived The concrete implementation class (CRTP parameter)
 * @tparam DataPacket Either SpiPacket or I2cPacket
 */
template <class Derived, class DataPacket>
class SyncPacketSender : NonCopyable<SyncPacketSender<Derived, DataPacket>> {
 public:
  static constexpr bool is_sync = true;
  /**
   * @brief Execute a packet transfer (blocking)
   * @param pkt Packet to transfer
   * @return true if the transfer succeeded, false on hardware error
   */
  [[nodiscard]] bool ProcessCommand(DataPacket &pkt) {
    return SendPacket(pkt);
  }

 private:
  Derived &derived() { return static_cast<Derived &>(*this); }

  bool SendPacket(DataPacket &pkt) {
    bool ok = true;
    derived().ChipSelect(pkt, true);
    switch (pkt.type) {
      case PacketType::Read:
        ok = derived().ReadPacket(pkt);
        break;
      case PacketType::WriteAndRead:
        ok = derived().ReadWritePacket(pkt);
        break;
      case PacketType::WriteThenRead:
        ok = derived().WritePacket(pkt);
        if (ok) ok = derived().ReadPacket(pkt);
        break;
      case PacketType::Write:
      default:
        ok = derived().WritePacket(pkt);
        break;
    }
    derived().ChipSelect(pkt, false);
    return ok;
  }
};

/// Synchronous SPI device CRTP base — usage: class MySpi : public Spi<MySpi> { ... };
template <typename Derived>
using Spi = SyncPacketSender<Derived, SpiPacket>;

/// Synchronous I2C device CRTP base — usage: class MyI2c : public I2c<MyI2c> { ... };
template <typename Derived>
using I2c = SyncPacketSender<Derived, I2cPacket>;

/// Asynchronous SPI device CRTP base — usage: class MySpi : public AsyncSpi<MySpi> { ... };
template <typename Derived, uint8_t QueueSize = 32>
using AsyncSpi = AsyncPacketSender<Derived, SpiPacket, QueueSize>;

/// Asynchronous I2C device CRTP base — usage: class MyI2c : public AsyncI2c<MyI2c> { ... };
template <typename Derived, uint8_t QueueSize = 32>
using AsyncI2c = AsyncPacketSender<Derived, I2cPacket, QueueSize>;

/**
 * @brief SPI peripheral device wrapper
 *
 * Stores a reference to the SPI transport and per-device configuration
 * (chip select, callback). Automatically applies configuration to each
 * packet before sending, so device-specific settings are set once in the
 * constructor rather than repeated on every transfer.
 *
 * @tparam Transport SPI transport type (concrete class inheriting Spi<> or AsyncSpi<>)
 */
template <class Transport>
class SpiDevice {
  Transport &transport_;
  ChipSelectPin chip_select_{};
  void (*callback_)(Data &){};
  SpiPolarity polarity_{SpiPolarity::Low};
  SpiPhase phase_{SpiPhase::Leading};

 public:
  /// Construct without chip select
  explicit SpiDevice(Transport &transport, void (*cb)(Data &) = nullptr,
                     SpiPolarity pol = SpiPolarity::Low, SpiPhase pha = SpiPhase::Leading)
      : transport_(transport), callback_(cb), polarity_(pol), phase_(pha) {}

  /// Construct with chip select (any GPIO type that has Set(bool))
  template <typename Gpio>
  SpiDevice(Transport &transport, Gpio &cs, void (*cb)(Data &) = nullptr,
            SpiPolarity pol = SpiPolarity::Low, SpiPhase pha = SpiPhase::Leading)
      : transport_(transport), chip_select_(cs), callback_(cb), polarity_(pol), phase_(pha) {}

  /// Set SPI clock polarity and phase
  void SetSpiMode(SpiPolarity pol, SpiPhase pha) {
    polarity_ = pol;
    phase_ = pha;
  }

  [[nodiscard]] bool ProcessCommand(SpiPacket &pkt) {
    pkt.chip_select = chip_select_;
    pkt.polarity = polarity_;
    pkt.phase = phase_;
    if (!pkt.callback) pkt.callback = callback_;
    return transport_.ProcessCommand(pkt);
  }

  /// Write-only transfer (sync transports only)
  [[nodiscard]] bool Write(Data &tx) {
    static_assert(Transport::is_sync, "Write() uses a stack-local packet; use ProcessCommand() with async transports");
    SpiPacket pkt{&tx};
    pkt.type = PacketType::Write;
    return ProcessCommand(pkt);
  }

  /// Read-only transfer (sync transports only)
  [[nodiscard]] bool Read(Data &rx) {
    static_assert(Transport::is_sync, "Read() uses a stack-local packet; use ProcessCommand() with async transports");
    SpiPacket pkt{nullptr, &rx};
    pkt.type = PacketType::Read;
    return ProcessCommand(pkt);
  }

  /// Full-duplex simultaneous write and read (sync transports only)
  [[nodiscard]] bool WriteAndRead(Data &tx, Data &rx) {
    static_assert(Transport::is_sync, "WriteAndRead() uses a stack-local packet; use ProcessCommand() with async transports");
    SpiPacket pkt{&tx, &rx};
    pkt.type = PacketType::WriteAndRead;
    return ProcessCommand(pkt);
  }

  /// Write first, then read (sync transports only)
  [[nodiscard]] bool WriteThenRead(Data &tx, Data &rx) {
    static_assert(Transport::is_sync, "WriteThenRead() uses a stack-local packet; use ProcessCommand() with async transports");
    SpiPacket pkt{&tx, &rx};
    pkt.type = PacketType::WriteThenRead;
    return ProcessCommand(pkt);
  }

  /// Access the underlying transport (e.g., to check last_status() / last_error())
  Transport &transport() { return transport_; }
  const Transport &transport() const { return transport_; }
};

/**
 * @brief Dual-mode SPI device wrapper (sync + async from one object)
 *
 * Combines a synchronous transport and an asynchronous transport behind a single
 * device interface. Both transports must share the same underlying SPI bus — it
 * is the caller's responsibility to ensure no overlapping transfers.
 *
 * Blocking convenience methods (Write, Read, WriteThenRead, WriteAndRead) use
 * the sync transport with stack-local packets. The async path
 * (ProcessCommand) uses the async transport with long-lived packets.
 *
 * Usage:
 * @code
 * Stm32SpiDevice        sync_spi(&hspi1);
 * Stm32DmaSpiDevice<>   dma_spi(&hspi1);
 *
 * DualSpiDevice sensor(sync_spi, dma_spi, cs_pin, my_async_callback);
 *
 * // Blocking single-register read during init
 * Buffer<1> tx, rx;
 * tx.Set(0x80 | WHO_AM_I);
 * rx.length = 1;
 * sensor.WriteThenRead(tx, rx);
 *
 * // Non-blocking bulk DMA read during runtime
 * static SpiPacketBundle<256> bulk;
 * bulk.pkt.type = PacketType::Read;
 * bulk.rx.length = 256;
 * sensor.ProcessCommand(bulk.pkt);
 * @endcode
 *
 * @tparam SyncTransport  Synchronous SPI transport (must satisfy is_sync == true)
 * @tparam AsyncTransport Asynchronous SPI transport (must satisfy is_sync == false)
 */
template <class SyncTransport, class AsyncTransport>
class DualSpiDevice {
  static_assert(SyncTransport::is_sync,   "SyncTransport must be a synchronous transport (is_sync == true)");
  static_assert(!AsyncTransport::is_sync,  "AsyncTransport must be an asynchronous transport (is_sync == false)");

  SyncTransport &sync_transport_;
  AsyncTransport &async_transport_;
  ChipSelectPin chip_select_{};
  void (*callback_)(Data &){};
  SpiPolarity polarity_{SpiPolarity::Low};
  SpiPhase phase_{SpiPhase::Leading};

  void StampPacket(SpiPacket &pkt) {
    pkt.chip_select = chip_select_;
    pkt.polarity = polarity_;
    pkt.phase = phase_;
    if (!pkt.callback) pkt.callback = callback_;
  }

 public:
  /// Construct without chip select
  DualSpiDevice(SyncTransport &sync, AsyncTransport &async,
                void (*cb)(Data &) = nullptr,
                SpiPolarity pol = SpiPolarity::Low, SpiPhase pha = SpiPhase::Leading)
      : sync_transport_(sync), async_transport_(async),
        callback_(cb), polarity_(pol), phase_(pha) {}

  /// Construct with chip select (any GPIO type that has Set(bool))
  template <typename Gpio>
  DualSpiDevice(SyncTransport &sync, AsyncTransport &async, Gpio &cs,
                void (*cb)(Data &) = nullptr,
                SpiPolarity pol = SpiPolarity::Low, SpiPhase pha = SpiPhase::Leading)
      : sync_transport_(sync), async_transport_(async),
        chip_select_(cs), callback_(cb), polarity_(pol), phase_(pha) {}

  /// Set SPI clock polarity and phase
  void SetSpiMode(SpiPolarity pol, SpiPhase pha) {
    polarity_ = pol;
    phase_ = pha;
  }

  // --- Async path (uses the async transport with long-lived packets) ---

  /// Queue an async transfer via the async transport (DMA/IT). Packet must outlive the transfer.
  [[nodiscard]] bool ProcessCommand(SpiPacket &pkt) {
    StampPacket(pkt);
    return async_transport_.ProcessCommand(pkt);
  }

  // --- Sync path (uses the sync transport with stack-local packets) ---

  /// Blocking write-only transfer
  [[nodiscard]] bool Write(Data &tx) {
    SpiPacket pkt{&tx};
    pkt.type = PacketType::Write;
    StampPacket(pkt);
    return sync_transport_.ProcessCommand(pkt);
  }

  /// Blocking read-only transfer
  [[nodiscard]] bool Read(Data &rx) {
    SpiPacket pkt{nullptr, &rx};
    pkt.type = PacketType::Read;
    StampPacket(pkt);
    return sync_transport_.ProcessCommand(pkt);
  }

  /// Blocking full-duplex simultaneous write and read
  [[nodiscard]] bool WriteAndRead(Data &tx, Data &rx) {
    SpiPacket pkt{&tx, &rx};
    pkt.type = PacketType::WriteAndRead;
    StampPacket(pkt);
    return sync_transport_.ProcessCommand(pkt);
  }

  /// Blocking write first, then read
  [[nodiscard]] bool WriteThenRead(Data &tx, Data &rx) {
    SpiPacket pkt{&tx, &rx};
    pkt.type = PacketType::WriteThenRead;
    StampPacket(pkt);
    return sync_transport_.ProcessCommand(pkt);
  }

  /// Access the sync transport (e.g., to check last_status())
  SyncTransport &sync_transport() { return sync_transport_; }
  const SyncTransport &sync_transport() const { return sync_transport_; }

  /// Access the async transport (e.g., to call TxRxCmpltCb(), check last_status())
  AsyncTransport &async_transport() { return async_transport_; }
  const AsyncTransport &async_transport() const { return async_transport_; }
};

/**
 * @brief I2C peripheral device wrapper
 *
 * Stores a reference to the I2C transport and per-device configuration
 * (address, callback). Automatically applies configuration to each packet
 * before sending, so device-specific settings are set once in the constructor
 * rather than repeated on every transfer.
 *
 * @note No WriteAndRead() convenience method — I2C is half-duplex.
 *       Use WriteThenRead() for register read patterns.
 *
 * @tparam Transport I2C transport type (concrete class inheriting I2c<> or AsyncI2c<>)
 */
template <class Transport>
class I2cDevice {
  Transport &transport_;
  uint8_t addr_{};
  void (*callback_)(Data &){};

 public:
  I2cDevice(Transport &transport, uint8_t addr, void (*cb)(Data &) = nullptr)
      : transport_(transport), addr_(addr), callback_(cb) {}

  [[nodiscard]] bool ProcessCommand(I2cPacket &pkt) {
    pkt.addr = addr_;
    if (!pkt.callback) pkt.callback = callback_;
    return transport_.ProcessCommand(pkt);
  }

  /// Write-only transfer (sync transports only)
  [[nodiscard]] bool Write(Data &tx) {
    static_assert(Transport::is_sync, "Write() uses a stack-local packet; use ProcessCommand() with async transports");
    I2cPacket pkt{&tx};
    pkt.type = PacketType::Write;
    return ProcessCommand(pkt);
  }

  /// Read-only transfer (sync transports only)
  [[nodiscard]] bool Read(Data &rx) {
    static_assert(Transport::is_sync, "Read() uses a stack-local packet; use ProcessCommand() with async transports");
    I2cPacket pkt{nullptr, &rx};
    pkt.type = PacketType::Read;
    return ProcessCommand(pkt);
  }

  /// Write first, then read (sync transports only)
  [[nodiscard]] bool WriteThenRead(Data &tx, Data &rx) {
    static_assert(Transport::is_sync, "WriteThenRead() uses a stack-local packet; use ProcessCommand() with async transports");
    I2cPacket pkt{&tx, &rx};
    pkt.type = PacketType::WriteThenRead;
    return ProcessCommand(pkt);
  }

  /// Access the underlying transport (e.g., to check last_status() / last_error())
  Transport &transport() { return transport_; }
  const Transport &transport() const { return transport_; }
};

/**
 * @brief Dual-mode I2C device wrapper (sync + async from one object)
 *
 * Same concept as DualSpiDevice but for I2C. Combines a blocking transport
 * for simple register reads and an async transport for DMA/interrupt bulk transfers.
 *
 * @tparam SyncTransport  Synchronous I2C transport (must satisfy is_sync == true)
 * @tparam AsyncTransport Asynchronous I2C transport (must satisfy is_sync == false)
 */
template <class SyncTransport, class AsyncTransport>
class DualI2cDevice {
  static_assert(SyncTransport::is_sync,   "SyncTransport must be a synchronous transport (is_sync == true)");
  static_assert(!AsyncTransport::is_sync,  "AsyncTransport must be an asynchronous transport (is_sync == false)");

  SyncTransport &sync_transport_;
  AsyncTransport &async_transport_;
  uint8_t addr_{};
  void (*callback_)(Data &){};

  void StampPacket(I2cPacket &pkt) {
    pkt.addr = addr_;
    if (!pkt.callback) pkt.callback = callback_;
  }

 public:
  DualI2cDevice(SyncTransport &sync, AsyncTransport &async,
                uint8_t addr, void (*cb)(Data &) = nullptr)
      : sync_transport_(sync), async_transport_(async),
        addr_(addr), callback_(cb) {}

  // --- Async path ---

  /// Queue an async transfer via the async transport. Packet must outlive the transfer.
  [[nodiscard]] bool ProcessCommand(I2cPacket &pkt) {
    StampPacket(pkt);
    return async_transport_.ProcessCommand(pkt);
  }

  // --- Sync path ---

  /// Blocking write-only transfer
  [[nodiscard]] bool Write(Data &tx) {
    I2cPacket pkt{&tx};
    pkt.type = PacketType::Write;
    StampPacket(pkt);
    return sync_transport_.ProcessCommand(pkt);
  }

  /// Blocking read-only transfer
  [[nodiscard]] bool Read(Data &rx) {
    I2cPacket pkt{nullptr, &rx};
    pkt.type = PacketType::Read;
    StampPacket(pkt);
    return sync_transport_.ProcessCommand(pkt);
  }

  /// Blocking write first, then read
  [[nodiscard]] bool WriteThenRead(Data &tx, Data &rx) {
    I2cPacket pkt{&tx, &rx};
    pkt.type = PacketType::WriteThenRead;
    StampPacket(pkt);
    return sync_transport_.ProcessCommand(pkt);
  }

  /// Access the sync transport (e.g., to check last_status())
  SyncTransport &sync_transport() { return sync_transport_; }
  const SyncTransport &sync_transport() const { return sync_transport_; }

  /// Access the async transport (e.g., to call TxRxCmpltCb(), check last_status())
  AsyncTransport &async_transport() { return async_transport_; }
  const AsyncTransport &async_transport() const { return async_transport_; }
};

}  // namespace myl_utils
