#pragma once

/**
 * @file peripheral.h
 * @brief Base classes for SPI and I2C peripheral communication
 *
 * This module provides abstract base classes for both synchronous and asynchronous
 * SPI/I2C communication. Sensors and other peripherals can use these interfaces
 * to work seamlessly with either sync or async implementations.
 *
 * Usage:
 * 1. Create a transport (e.g., ZephyrSpiDevice, ZephyrI2cDevice)
 * 2. Wrap it in an SpiDevice/I2cDevice with per-device config (addr, CS, callback)
 * 3. Create Buffer objects and use Set()/Append() to populate TX data
 * 4. Use convenience methods (Write, Read, WriteThenRead) or ProcessCommand()
 */

#include "buffer.h"
#include "gpio.h"
#include "myl_utils/noncopyable.h"
#include <cstdint>

/**
 * @brief Specifies the type of SPI/I2C transfer to perform
 */
enum class PacketType {
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

  /// Reset buffer and write bytes, updating length automatically. Zeros remaining storage.
  template <typename... Bytes>
  void Set(Bytes... bytes) {
    static_assert(sizeof...(bytes) <= Size, "Too many bytes for buffer");
    uint8_t tmp[] = {static_cast<uint8_t>(bytes)...};
    for (uint16_t i = 0; i < sizeof...(bytes); ++i) storage[i] = tmp[i];
    for (uint16_t i = sizeof...(bytes); i < Size; ++i) storage[i] = 0;
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
 * @brief SPI transfer packet containing buffer pointers and transfer configuration
 *
 * Buffers are optional — set only tx_data for write-only, only rx_data for read-only,
 * or both for bidirectional transfers. Per-device fields (chip_select, callback) are
 * typically set automatically by the SpiDevice wrapper.
 */
struct SpiPacket {
  Data *tx_data{};               ///< TX buffer (nullptr if unused)
  Data *rx_data{};               ///< RX buffer (nullptr if unused)
  PacketType type{PacketType::Write};
  GpioOutput *chip_select{};     ///< Optional chip select GPIO (active = asserted)
  void (*callback)(Data &){};    ///< Optional completion callback

  /// Assign packet type directly: pkt = PacketType::WriteThenRead;
  SpiPacket &operator=(PacketType t) { type = t; return *this; }

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

  /// Assign packet type directly: pkt = PacketType::WriteThenRead;
  I2cPacket &operator=(PacketType t) { type = t; return *this; }

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
  Buffer<size> name##_tx_;        \
  Buffer<size> name##_rx_;

#define I2C_PACKET_HELPER(name, size) \
  PACKET_HELPER(name, size)           \
  I2cPacket name{&name##_tx_, &name##_rx_};

#define SPI_PACKET_HELPER(name, size) \
  PACKET_HELPER(name, size)           \
  SpiPacket name{&name##_tx_, &name##_rx_};

/**
 * @brief Abstract base class for asynchronous packet-based transfers
 *
 * Provides queuing functionality for non-blocking transfers. When a transfer
 * completes, call TxRxCmpltCb() from the interrupt handler to process the
 * next queued packet.
 *
 * @note ISR safety assumes ARM Cortex-M memory model (single-core, strongly-ordered
 *       peripheral writes). The volatile qualifier on busy_ and the circular buffer
 *       indices is sufficient on this architecture without explicit memory barriers.
 *
 * @tparam DataPacket Either SpiPacket or I2cPacket
 * @tparam QueueSize Maximum number of packets that can be queued (default: 25)
 */
template <class DataPacket, uint8_t QueueSize = 25>
class AsyncPacketSender : NonCopyable<AsyncPacketSender<DataPacket, QueueSize>> {
 protected:
  /// @note Access from both thread and ISR context — volatile is sufficient on ARM Cortex-M
  volatile bool busy_{};

  static constexpr bool is_sync = false;

  /**
   * @brief Perform simultaneous read and write (full-duplex)
   * @param pkt Packet containing TX and RX data
   */
  virtual void ReadWritePacket(DataPacket &pkt) = 0;

  /**
   * @brief Perform write operation
   * @param pkt Packet containing TX data
   */
  virtual void WritePacket(DataPacket &pkt) = 0;

  /**
   * @brief Perform read operation
   * @param pkt Packet containing RX buffer
   */
  virtual void ReadPacket(DataPacket &pkt) = 0;

  /**
   * @brief Control chip select line (SPI only, I2C implementations can leave empty)
   * @param pkt The current packet
   * @param enable true to assert CS (active), false to deassert
   */
  virtual void ChipSelect(DataPacket &pkt, bool enable) = 0;

 public:
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
   */
  void TxRxCmpltCb() {
    if (current_command_->type == PacketType::WriteThenRead && write_and_read_pkt_) {
      write_and_read_pkt_ = false;
      ReadPacket(*current_command_);
      return;
    }

    ChipSelect(*current_command_, false);
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

  bool QueuePacket(DataPacket &pkt) {
    if (busy_) {
      if (!pkt_queue_.Writable()) return false;
      pkt_queue_.Put(&pkt);
      return true;
    } else {
      SendPacket(pkt);
      return true;
    }
  }

  void SendPacket(DataPacket &pkt) {
    busy_ = true;
    write_and_read_pkt_ = false;
    current_command_ = &pkt;
    ChipSelect(pkt, true);
    switch (pkt.type) {
      case PacketType::Read:
        ReadPacket(pkt);
        break;
      case PacketType::WriteAndRead:
        ReadWritePacket(pkt);
        break;
      case PacketType::WriteThenRead:
        write_and_read_pkt_ = true;
        WritePacket(pkt);
        break;
      case PacketType::Write:
      default:
        WritePacket(pkt);
        break;
    }
  }
};

/**
 * @brief Abstract base class for synchronous packet-based transfers
 *
 * Provides blocking transfer functionality. Each call to ProcessCommand()
 * completes the entire transfer before returning.
 *
 * @tparam DataPacket Either SpiPacket or I2cPacket
 */
template <class DataPacket>
class SyncPacketSender : NonCopyable<SyncPacketSender<DataPacket>> {
 protected:
  static constexpr bool is_sync = true;
  /**
   * @brief Perform simultaneous read and write (full-duplex)
   * @param pkt Packet containing TX and RX data
   */
  virtual void ReadWritePacket(DataPacket &pkt) = 0;

  /**
   * @brief Perform write operation
   * @param pkt Packet containing TX data
   */
  virtual void WritePacket(DataPacket &pkt) = 0;

  /**
   * @brief Perform read operation
   * @param pkt Packet containing RX buffer
   */
  virtual void ReadPacket(DataPacket &pkt) = 0;

  /**
   * @brief Control chip select line (SPI only, I2C implementations can leave empty)
   * @param pkt The current packet
   * @param enable true to assert CS (active), false to deassert
   */
  virtual void ChipSelect(DataPacket &pkt, bool enable) = 0;

 public:
  /**
   * @brief Execute a packet transfer (blocking)
   * @param pkt Packet to transfer
   * @return Always true (sync transfers cannot fail to queue)
   */
  bool ProcessCommand(DataPacket &pkt) {
    SendPacket(pkt);
    return true;
  }

 private:
  void SendPacket(DataPacket &pkt) {
    ChipSelect(pkt, true);
    switch (pkt.type) {
      case PacketType::Read:
        ReadPacket(pkt);
        break;
      case PacketType::WriteAndRead:
        ReadWritePacket(pkt);
        break;
      case PacketType::WriteThenRead:
        WritePacket(pkt);
        ReadPacket(pkt);
        break;
      case PacketType::Write:
      default:
        WritePacket(pkt);
        break;
    }
    ChipSelect(pkt, false);
  }
};

/// Synchronous SPI device base class
using Spi = SyncPacketSender<SpiPacket>;
/// Synchronous I2C device base class
using I2c = SyncPacketSender<I2cPacket>;
/// Asynchronous SPI device base class
using AsyncSpi = AsyncPacketSender<SpiPacket>;
/// Asynchronous I2C device base class
using AsyncI2c = AsyncPacketSender<I2cPacket>;

/**
 * @brief SPI peripheral device wrapper
 *
 * Stores a reference to the SPI transport and per-device configuration
 * (chip select, callback). Automatically applies configuration to each
 * packet before sending, so device-specific settings are set once in the
 * constructor rather than repeated on every transfer.
 *
 * @tparam Transport SPI transport type (Spi or AsyncSpi)
 */
template <class Transport>
class SpiDevice {
  Transport &transport_;
  GpioOutput *chip_select_{};
  void (*callback_)(Data &){};

 public:
  SpiDevice(Transport &transport, GpioOutput *cs = nullptr, void (*cb)(Data &) = nullptr)
      : transport_(transport), chip_select_(cs), callback_(cb) {}

  [[nodiscard]] bool ProcessCommand(SpiPacket &pkt) {
    pkt.chip_select = chip_select_;
    if (!pkt.callback) pkt.callback = callback_;
    return transport_.ProcessCommand(pkt);
  }

  /// Write-only transfer (sync transports only)
  void Write(Data &tx) {
    static_assert(Transport::is_sync, "Write() uses a stack-local packet; use ProcessCommand() with async transports");
    SpiPacket pkt{&tx};
    pkt.type = PacketType::Write;
    ProcessCommand(pkt);
  }

  /// Read-only transfer (sync transports only)
  void Read(Data &rx) {
    static_assert(Transport::is_sync, "Read() uses a stack-local packet; use ProcessCommand() with async transports");
    SpiPacket pkt{nullptr, &rx};
    pkt.type = PacketType::Read;
    ProcessCommand(pkt);
  }

  /// Full-duplex simultaneous write and read (sync transports only)
  void WriteAndRead(Data &tx, Data &rx) {
    static_assert(Transport::is_sync, "WriteAndRead() uses a stack-local packet; use ProcessCommand() with async transports");
    SpiPacket pkt{&tx, &rx};
    pkt.type = PacketType::WriteAndRead;
    ProcessCommand(pkt);
  }

  /// Write first, then read (sync transports only)
  void WriteThenRead(Data &tx, Data &rx) {
    static_assert(Transport::is_sync, "WriteThenRead() uses a stack-local packet; use ProcessCommand() with async transports");
    SpiPacket pkt{&tx, &rx};
    pkt.type = PacketType::WriteThenRead;
    ProcessCommand(pkt);
  }
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
 * @tparam Transport I2C transport type (I2c or AsyncI2c)
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
  void Write(Data &tx) {
    static_assert(Transport::is_sync, "Write() uses a stack-local packet; use ProcessCommand() with async transports");
    I2cPacket pkt{&tx};
    pkt.type = PacketType::Write;
    ProcessCommand(pkt);
  }

  /// Read-only transfer (sync transports only)
  void Read(Data &rx) {
    static_assert(Transport::is_sync, "Read() uses a stack-local packet; use ProcessCommand() with async transports");
    I2cPacket pkt{nullptr, &rx};
    pkt.type = PacketType::Read;
    ProcessCommand(pkt);
  }

  /// Write first, then read (sync transports only)
  void WriteThenRead(Data &tx, Data &rx) {
    static_assert(Transport::is_sync, "WriteThenRead() uses a stack-local packet; use ProcessCommand() with async transports");
    I2cPacket pkt{&tx, &rx};
    pkt.type = PacketType::WriteThenRead;
    ProcessCommand(pkt);
  }
};
