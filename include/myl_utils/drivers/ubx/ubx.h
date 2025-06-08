#pragma once

#include <myl_utils/serial.h>

#include "messages.h"

enum UBX_PARSER_STATUS {
  UBX_PARSER_NO_MESSAGE,
  UBX_PARSER_FRAMING_FAILURE,
  UBX_PARSER_MESSAGE_OVERSIZE,
  UBX_PARSER_CHECKSUM_FAILURE,
  UBX_PARSER_UNIMPLEMENTED_CLASS_ID,
  UBX_PARSER_UNKNOWN_MESSAGE_ID,
  UBX_PARSER_UNKNOWN_CLASS_ID,
  UBX_PARSER_GOT_ACCEL_MESSAGE,
  UBX_PARSER_GOT_GYRO_MESSAGE,
  UBX_PARSER_GOT_MAG_MESSAGE,
  UBX_PARSER_GOT_AMB_PRESSURE_MESSAGE,
};

enum UBX_PARSER_STATE {
  WAITING_FOR_SYNC1,
  WAITING_FOR_SYNC2,
  WAITING_FOR_CLASS_ID,
  WAITING_FOR_MSG_ID,
  WAITING_FOR_LENGTH1,
  WAITING_FOR_LENGTH2,
  WAITING_FOR_PAYLOAD_AND_CHECKSUM
};

const uint8_t UBX_PACKET_SYNC1_LOC = 0;
const uint8_t UBX_PACKET_SYNC2_LOC = 1;
const uint8_t UBX_PACKET_CLASS_ID_LOC = 2;
const uint8_t UBX_PACKET_MSG_ID_LOC = 3;
const uint8_t UBX_PACKET_LENGTH_LOC = 4;
const uint8_t UBX_PACKET_PAYLOAD_LOC = 6;

const uint8_t UBX_PACKET_CHECKSUM_LENGTH = 2;
const uint8_t UBX_PACKET_NON_PAYLOAD_LENGTH = UBX_PACKET_PAYLOAD_LOC + UBX_PACKET_CHECKSUM_LENGTH;
const uint16_t UBX_PARSER_MAX_BUFFER_SIZE = 1024;

class UbxDevice {
 public:
  UbxDevice(SerialPort& port) : port_(port) {}

  void Runner() {
    if (port_.Readable()) {
      auto status = ubx_parse_byte(port_.GetC());
      printk("UBX Parser Status %d /n", status);
    }
  }

  uint16_t calc_checksum(uint8_t* data, uint16_t length) {
    uint8_t CK_A = 0, CK_B = 0;
    for (size_t i = 0; i < length; i++) {
      uint8_t d = data[i];
      CK_A = CK_A + d;
      CK_B = CK_B + CK_A;
    }
    return CK_A << 8 | CK_B;
  }

  void send_message(uint16_t msg_id, uint8_t* data, uint16_t length) {
    uint8_t buffer[1024];
    buffer[UBX_PACKET_SYNC1_LOC] = UBX_SYNC1;
    buffer[UBX_PACKET_SYNC2_LOC] = UBX_SYNC2;
    buffer[UBX_PACKET_CLASS_ID_LOC] = msg_id >> 8;
    buffer[UBX_PACKET_MSG_ID_LOC] = msg_id;
    buffer[UBX_PACKET_LENGTH_LOC] = length;
    buffer[UBX_PACKET_LENGTH_LOC + 1] = length >> 8;
    memcpy(buffer + UBX_PACKET_PAYLOAD_LOC, data, length);
    uint16_t checksum =
        calc_checksum(buffer + UBX_PACKET_CLASS_ID_LOC, length + (UBX_PACKET_PAYLOAD_LOC - UBX_PACKET_CLASS_ID_LOC));
    buffer[length + 6] = checksum >> 8;
    buffer[length + 7] = checksum;
    for (int i = 0; i < length + 8; i++) {
      port_.PutC(buffer[i]);
    }
  }

  void get_version() { send_message((uint16_t)UBX_MON_MSG_ID::UBX_VERSION_GET, nullptr, 0); }

  void configure(ubx_keys_msg_out key, uint8_t* data, uint8_t length) {
    uint8_t buffer[4 + 4 + length];
    buffer[0] = 0;
    buffer[1] = 1;
    buffer[2] = 0;
    buffer[3] = 0;
    memcpy(buffer + 4, (uint8_t*)&key, 4);
    memcpy(buffer + 8, data, length);
    send_message((uint16_t)UBX_CONFIG_MSG_ID::UBX_CFG_VALSET, buffer, 4 + 4 + length);
  }

  void configure_uint8(ubx_keys_msg_out key, uint8_t data) { configure(key, &data, 1); }
  void configure_uint16(ubx_keys_msg_out key, uint16_t data) { configure(key, (uint8_t*)&data, 2); }
  void configure_uint32(ubx_keys_msg_out key, uint32_t data) { configure(key, (uint8_t*)&data, 4); }

  void initialize_messages() {
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_GGA_UART1, 0);
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_RMC_UART1, 0);
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_GSV_UART1, 0);
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_DTM_UART1, 0);
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_GBS_UART1, 0);
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_GLL_UART1, 0);
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_GNS_UART1, 0);

    configure_uint8(UBX_KEY_MSG_OUT_NMEA_GRS_UART1, 0);
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_GSA_UART1, 0);
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_GST_UART1, 0);
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_VLW_UART1, 0);
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_VTG_UART1, 0);
    configure_uint8(UBX_KEY_MSG_OUT_NMEA_ZDA_UART1, 0);

    configure_uint8(UBX_KEY_MSG_OUT_UBX_NAV_PVT_UART1, 1);
    configure_uint8(UBX_KEY_NAV_CFG_FIX_MODE, (uint8_t)ubx_fix_mode::UBX_FIX_MODE_AUTO);
  }

  enum UBX_PARSER_STATUS process_config_packet(uint8_t packet_length) { return UBX_PARSER_UNKNOWN_MESSAGE_ID; }

  enum UBX_PARSER_STATUS process_full_packet(uint8_t packet_length) {
    uint16_t calculated_checksum = calc_checksum(rx_buffer_ + 2, packet_length + 4);
    if (calculated_checksum !=
        (((uint16_t)rx_buffer_[packet_length - 2] << 8) + (uint16_t)rx_buffer_[packet_length - 1])) {
      return UBX_PARSER_CHECKSUM_FAILURE;
    }

    switch (rx_buffer_[UBX_PACKET_CLASS_ID_LOC]) {
      case UBX_CONFIG:
        return process_config_packet(packet_length);
        break;

      case UBX_NAV:
      case UBX_RXM:
      case UBX_INF:
      case UBX_ACK:

      case UBX_UPD:
      case UBX_MON:
      case UBX_TIM:
      case UBX_MGA:
      case UBX_LOG:
      case UBX_SEC:
      case UBX_NAV2:
        return UBX_PARSER_UNIMPLEMENTED_CLASS_ID;

      default:
        break;
    }
    return UBX_PARSER_UNKNOWN_CLASS_ID;
  }

  enum UBX_PARSER_STATUS ubx_parse_byte(uint8_t byte) {
    switch (parser_state_) {
      case WAITING_FOR_SYNC1:
        byte_index_ = 0;
        if (byte == UBX_SYNC1) {
          rx_buffer_[byte_index_] = byte;
          byte_index_++;
          parser_state_ = WAITING_FOR_SYNC2;
        }
        return UBX_PARSER_NO_MESSAGE;
        break;

      case WAITING_FOR_SYNC2:
        if (byte == UBX_SYNC2) {
          rx_buffer_[byte_index_] = byte;
          byte_index_++;
          parser_state_ = WAITING_FOR_CLASS_ID;
          return UBX_PARSER_NO_MESSAGE;
        }
        break;

      case WAITING_FOR_CLASS_ID:
        rx_buffer_[byte_index_] = byte;
        byte_index_++;
        parser_state_ = WAITING_FOR_MSG_ID;
        return UBX_PARSER_NO_MESSAGE;
        break;

      case WAITING_FOR_MSG_ID:
        rx_buffer_[byte_index_] = byte;
        byte_index_++;
        parser_state_ = WAITING_FOR_LENGTH1;
        return UBX_PARSER_NO_MESSAGE;
        break;

      case WAITING_FOR_LENGTH1:
        rx_buffer_[byte_index_] = byte;
        byte_index_++;
        parser_state_ = WAITING_FOR_LENGTH2;
        return UBX_PARSER_NO_MESSAGE;
        break;

      case WAITING_FOR_LENGTH2:
        packet_length_ = (rx_buffer_[UBX_PACKET_LENGTH_LOC] << 8 | byte) + UBX_PACKET_NON_PAYLOAD_LENGTH;
        if (packet_length_ > UBX_PARSER_MAX_BUFFER_SIZE) {
          parser_state_ = WAITING_FOR_SYNC1;
          return UBX_PARSER_MESSAGE_OVERSIZE;
        }
        rx_buffer_[byte_index_] = byte;
        byte_index_++;
        parser_state_ = WAITING_FOR_PAYLOAD_AND_CHECKSUM;
        return UBX_PARSER_NO_MESSAGE;
        break;

      case WAITING_FOR_PAYLOAD_AND_CHECKSUM:
        rx_buffer_[byte_index_] = byte;
        byte_index_++;
        if (byte_index_ == packet_length_) {
          parser_state_ = WAITING_FOR_SYNC1;
          return process_full_packet(packet_length_);
        }
        return UBX_PARSER_NO_MESSAGE;
        break;

      default:
        parser_state_ = WAITING_FOR_SYNC1;
        break;
    }
    return UBX_PARSER_FRAMING_FAILURE;
  }

 private:
  SerialPort& port_;
  uint8_t rx_buffer_[1024];
  enum UBX_PARSER_STATE parser_state_ = WAITING_FOR_SYNC1;
  uint8_t byte_index_ = 0;
  uint8_t packet_length_ = 0;
};