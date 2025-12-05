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
 * 1. Create a packet using SPI_PACKET_HELPER or I2C_PACKET_HELPER macros
 * 2. Set the packet type (Read, Write, WriteAndRead, or WriteThenRead)
 * 3. Call ProcessCommand() to execute the transfer
 */

#include "buffer.h"
#include "myl_utils/noncopyable.h"
#include "stdint.h"

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
 * @brief Simple data buffer structure for packet transfers
 */
struct Data {
  uint8_t *data;
  uint8_t length;
  explicit Data(uint8_t *ptr) : data{ptr}, length{0} {}
};

/**
 * @brief SPI transfer packet containing TX/RX buffers and transfer configuration
 */
struct SpiPacket {
  Data &tx_data;
  Data &rx_data;
  PacketType type;
  void (*chip_select)(bool);   ///< Optional chip select callback (enable/disable)
  void (*callback)(Data &);    ///< Optional completion callback

  SpiPacket(Data &tx_data, Data &rx_data)
      : tx_data(tx_data), rx_data(rx_data), type{PacketType::Write}, chip_select{nullptr}, callback{nullptr} {}
};

/**
 * @brief I2C transfer packet containing address, TX/RX buffers, and transfer configuration
 */
struct I2cPacket {
  uint8_t addr;
  Data &tx_data;
  Data &rx_data;
  PacketType type;
  void (*callback)(Data &);    ///< Optional completion callback

  I2cPacket(uint8_t addr, Data &tx_data, Data &rx_data)
      : addr(addr), tx_data(tx_data), rx_data(rx_data), type{PacketType::Write}, callback{nullptr} {}
};

/**
 * @brief Helper macro to create TX/RX buffers and Data objects
 * @param name Base name for the buffers and data objects
 * @param size Size of the TX and RX buffers
 */
#define PACKET_HELPER(name, size)          \
  uint8_t name##_tx_buffer_[size];         \
  Data name##_tx_data_{name##_tx_buffer_}; \
  uint8_t name##_rx_buffer_[size];         \
  Data name##_rx_data_{name##_rx_buffer_};

/**
 * @brief Helper macro to create an I2C packet with buffers
 * @param addr I2C device address
 * @param name Base name for the packet and buffers
 * @param size Size of the TX and RX buffers
 */
#define I2C_PACKET_HELPER(addr, name, size) \
  PACKET_HELPER(name, size)                 \
  I2cPacket name{addr, name##_tx_data_, name##_rx_data_};

/**
 * @brief Helper macro to create an SPI packet with buffers
 * @param name Base name for the packet and buffers
 * @param size Size of the TX and RX buffers
 */
#define SPI_PACKET_HELPER(name, size) \
  PACKET_HELPER(name, size)           \
  SpiPacket name{name##_tx_data_, name##_rx_data_};

/**
 * @brief Abstract base class for asynchronous packet-based transfers
 *
 * Provides queuing functionality for non-blocking transfers. When a transfer
 * completes, call TxRxCmpltCb() from the interrupt handler to process the
 * next queued packet.
 *
 * @tparam DataPacket Either SpiPacket or I2cPacket
 */
template <class DataPacket>
class AsyncPacketSender : NonCopyable<AsyncPacketSender<DataPacket>> {
 protected:
  bool busy_{};

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
   */
  void ProcessCommand(DataPacket &pkt) { QueuePacket(pkt); }

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
    if (current_command_->callback) {
      current_command_->callback(current_command_->rx_data);
    }

    if (pkt_queue_.Readable()) {
      DataPacket &temp_pkt_holder = *pkt_queue_.Get();
      SendPacket(temp_pkt_holder);
    } else {
      busy_ = false;
    }
  }

 private:
  CircularBuffer<DataPacket *, 25> pkt_queue_;
  bool write_and_read_pkt_{};
  DataPacket *current_command_;

  void QueuePacket(DataPacket &pkt) {
    if (busy_) {
      pkt_queue_.Put(&pkt);
    } else {
      SendPacket(pkt);
    }
  }

  void SendPacket(DataPacket &pkt) {
    busy_ = true;
    write_and_read_pkt_ = false;
    current_command_ = &pkt;
    ChipSelect(pkt, true);
    if (pkt.type == PacketType::Read) {
      ReadPacket(pkt);
    } else if (pkt.type == PacketType::WriteAndRead) {
      ReadWritePacket(pkt);
    } else if (pkt.type == PacketType::WriteThenRead) {
      write_and_read_pkt_ = true;
      WritePacket(pkt);
    } else {
      WritePacket(pkt);
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
   */
  void ProcessCommand(DataPacket &pkt) { SendPacket(pkt); }

 private:
  void SendPacket(DataPacket &pkt) {
    ChipSelect(pkt, true);
    if (pkt.type == PacketType::Read) {
      ReadPacket(pkt);
    } else if (pkt.type == PacketType::WriteAndRead) {
      ReadWritePacket(pkt);
    } else if (pkt.type == PacketType::WriteThenRead) {
      WritePacket(pkt);
      ReadPacket(pkt);
    } else {
      WritePacket(pkt);
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
