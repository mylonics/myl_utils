#pragma once

#include <functional>

#include "buffer.h"
#include "stdint.h"
#include "utils/noncopyable.h"

enum class PacketType { Read, Write, WriteAndRead, WriteThenRead };

struct Data {
  uint8_t *data;
  uint8_t length;
  Data(uint8_t *ptr) : data{ptr} {};
};

struct SpiPacket {
  Data &tx_data;
  Data &rx_data;
  PacketType type;
  std::function<void(bool)> chip_select = 0;
  std::function<void(Data &)> callback = 0;

  SpiPacket(Data &tx_data, Data &rx_data) : tx_data(tx_data), rx_data(rx_data){};
};

struct I2cPacket {
  uint8_t addr;
  Data &tx_data;
  Data &rx_data;
  PacketType type;
  std::function<void(Data &)> callback = 0;

  I2cPacket(uint8_t addr, Data &tx_data, Data &rx_data) : addr(addr), tx_data(tx_data), rx_data(rx_data){};
};

#define PACKET_HELPER(name, size)          \
  uint8_t name##_tx_buffer_[size];         \
  Data name##_tx_data_{name##_tx_buffer_}; \
  uint8_t name##_rx_buffer_[size];         \
  Data name##_rx_data_{name##_rx_buffer_};

#define I2C_PACKET_HELPER(addr, name, size) \
  PACKET_HELPER(name, size)                 \
  I2cPacket name{addr, name##_tx_data_, name##_rx_data_};

#define SPI_PACKET_HELPER(name, size) \
  PACKET_HELPER(name, size)           \
  SpiPacket name{name##_tx_data_, name##_rx_data_};

template <class DataPacket>
class AsyncPacketSender : NonCopyable<AsyncPacketSender<DataPacket>> {
 protected:
  bool busy_{};

 public:
  void ProcessCommand(DataPacket &pkt) { QueuePacket(pkt); }

  void TxRxCmpltCb() {
    if (current_command_->type == PacketType::WriteThenRead && write_and_read_pkt_) {
      write_and_read_pkt_ = false;
      ReadPacketIt(*current_command_);
      return;
    }

    chip_select(*current_command_, false);
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
    chip_select(pkt, true);
    if (pkt.type == PacketType::Read) {
      ReadPacketIt(pkt);
    } else if (pkt.type == PacketType::WriteAndRead) {
      ReadWritePacketIt(pkt);
    } else if (pkt.type == PacketType::WriteThenRead) {
      write_and_read_pkt_ = true;
      WritePacketIt(pkt);
    } else {
      WritePacketIt(pkt);
    }
  }
  virtual void chip_select(DataPacket &, bool) = 0;

  virtual void ReadWritePacketIt(DataPacket &) = 0;

  virtual void WritePacketIt(DataPacket &) = 0;

  virtual void ReadPacketIt(DataPacket &) = 0;
};

template <class DataPacket>
class SyncPacketSender : NonCopyable<SyncPacketSender<DataPacket>> {
 public:
  void ProcessCommand(DataPacket &pkt) { SendPacket(pkt); }

 private:
  void SendPacket(DataPacket &pkt) {
    chip_select(pkt, true);
    if (pkt.type == PacketType::Read) {
      ReadPacketIt(pkt);
    } else if (pkt.type == PacketType::WriteAndRead) {
      ReadWritePacketIt(pkt);
    } else if (pkt.type == PacketType::WriteThenRead) {
      WritePacketIt(pkt);
      ReadPacketIt(pkt);
    } else {
      WritePacketIt(pkt);
    }
    chip_select(pkt, false);
  }

  virtual void chip_select(DataPacket &, bool) = 0;

  virtual void ReadWritePacketIt(DataPacket &) = 0;

  virtual void WritePacketIt(DataPacket &) = 0;

  virtual void ReadPacketIt(DataPacket &) = 0;
};

typedef SyncPacketSender<SpiPacket> Spi;
typedef SyncPacketSender<I2cPacket> I2c;
