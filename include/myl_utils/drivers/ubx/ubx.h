#pragma once

#include <myl_utils/log.h>
#include <myl_utils/serial.h>

#include "messages.h"

enum UBX_PARSER_STATUS {
  UBX_PARSER_SYNCING,
  UBX_PARSER_FRAMING_FAILURE,
  UBX_PARSER_MESSAGE_OVERSIZE,
  UBX_PARSER_CHECKSUM_FAILURE,
  UBX_PARSER_NO_MESSAGE,
  UBX_PARSER_UNIMPLEMENTED_CLASS_ID,
  UBX_PARSER_UNKNOWN_MESSAGE_ID,
  UBX_PARSER_UNKNOWN_CLASS_ID,
  UBX_PARSER_PROCESSED_MSG
};

enum UBX_PARSER_STATE {
  WAITING_FOR_SYNC1,
  WAITING_FOR_SYNC2,
  WAITING_FOR_CLASS_ID,
  WAITING_FOR_MSG_ID,
  WAITING_FOR_LENGTH1,
  WAITING_FOR_LENGTH2,
  WAITING_FOR_PAYLOAD_AND_CHECKSUM,
  WAITING_FOR_RTCM_LENGTH1,
  WAITING_FOR_RTCM_LENGTH2,
  WAITING_FOR_RTCM_PAYLOAD_AND_CHECKSUM,
};

const uint8_t UBX_PACKET_SYNC1_LOC = 0;
const uint8_t UBX_PACKET_SYNC2_LOC = 1;
const uint8_t UBX_PACKET_CLASS_ID_LOC = 2;
const uint8_t UBX_PACKET_MSG_ID_LOC = 3;
const uint8_t UBX_PACKET_LENGTH_LOC = 4;
const uint8_t UBX_PACKET_PAYLOAD_LOC = 6;

const uint8_t RTCM_PACKET_LENGTH_LOC = 1;
const uint8_t RTCM_PACKET_NON_PAYLOAD_LENGTH = 6;
const uint8_t RTCM_PACKET_PAYLOAD_ID_LOC = 3;

const uint8_t UBX_PACKET_CHECKSUM_LENGTH = 2;
const uint8_t UBX_PACKET_NON_PAYLOAD_LENGTH = UBX_PACKET_PAYLOAD_LOC + UBX_PACKET_CHECKSUM_LENGTH;
const uint16_t UBX_PARSER_MAX_BUFFER_SIZE = 1024;

typedef void (*ubx_nav_pvt_cb_t)(ubx_nav_pvt nav_data);
typedef void (*ubx_rtcm_cb_t)(uint8_t msg_type, uint8_t* data, size_t length);

struct ubx_callbacks {
  ubx_nav_pvt_cb_t ubx_nav_pvt_cb{};
  ubx_rtcm_cb_t ubx_rtcm_cb{};
};

class UbxDevice {
 public:
  UbxDevice(SerialPort& port, ubx_callbacks cbs) : port_(port), cbs_{cbs} {
    port_.ReconfigureUart(UartBaud::B_38400, UartParity::NONE, UartStopBits::ONE);
  }

  void Runner() {
    if (port_.Readable()) {
      auto status = ubx_parse_byte(port_.GetC());
      if (status <= UBX_PARSER_CHECKSUM_FAILURE) {
        DECLARE_MYL_UTILS_LOG();
        LOG_ERR("Parsing Error %d \n", status);
      }
    }
  }

  void PortForwardRunner(SerialPort& fwd_port) {
    static uint8_t test_buffer[4096];
    while (fwd_port.Readable()) {
      size_t length = fwd_port.GetArray(test_buffer, 4096);
      port_.PutArray(test_buffer, length);
    }
    while (port_.Readable()) {
      uint8_t byte = port_.GetC();
      fwd_port.PutC(byte);
      auto status = ubx_parse_byte(byte);
      if (status <= UBX_PARSER_CHECKSUM_FAILURE) {
        DECLARE_MYL_UTILS_LOG();
        LOG_ERR("Parsing Error %d \n", status);
      }
    }
  };

  void SendRawData(uint8_t* data, size_t length) { port_.PutArray(data, length); }

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

  void get_version() { send_message(UBX_VERSION_GET_CLASS_ID, nullptr, 0); }

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

  void initialize_navigation() {
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

    configure_uint8(UBLOX_CFG_UART1INPROT_UBX, 1);
    configure_uint8(UBLOX_CFG_UART1INPROT_RTCM3X, 1);
    configure_uint8(UBLOX_CFG_MSGOUT_UBX_RXM_RTCM_UART1, 1);
    configure_uint8(UBLOX_CFG_MSGOUT_UBX_RXM_COR_UART1, 1);

    configure_uint8(UBLOX_CFG_TMODE_MODE, (uint8_t)ubx_survey_mode::DISABLED);
  }

  void initialize_rtcm_output(bool minimal_rtk = true) {
    initialize_navigation();

    if (minimal_rtk) {
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART1, 10);  // Base station ARP
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_UART1, 1);   // GPS MSM4
    } else {
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART1, 5);  // Base station ARP
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_UART1, 1);  // GPS MSM4
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 1);  // GPS MSM7
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_UART1, 1);  // GLONASS MSM4
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 1);  // GLONASS MSM7
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_UART1, 1);  // Galileo MSM4
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, 1);  // Galileo MSM7
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_UART1, 1);  // BeiDou MSM4
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, 1);  // BeiDou MSM7
      configure_uint8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 1);  // GLONASS L1 and L2 code-phase biases
    }

    configure_uint8(UBLOX_CFG_UART1OUTPROT_UBX, 1);
    // configure_uint8(UBLOX_CFG_UART1OUTPROT_RTCM3X, 1);
    startSurvey(60, 1.0);
  }

  void startSurvey(uint32_t observationTimeSeconds, double requiredAccuracyMeters) {
    configure_uint32(UBLOX_CFG_TMODE_SVIN_MIN_DUR, observationTimeSeconds);
    configure_uint32(UBLOX_CFG_TMODE_SVIN_ACC_LIMIT, requiredAccuracyMeters * 10000.0);

    configure_uint8(UBLOX_CFG_TMODE_MODE, (uint8_t)ubx_survey_mode::SURVEY_IN);
  }

  enum UBX_PARSER_STATUS process_config_packet() { return UBX_PARSER_UNKNOWN_MESSAGE_ID; }

  enum UBX_PARSER_STATUS process_mon_packet() {
    DECLARE_MYL_UTILS_LOG();
    switch (rx_buffer_[UBX_PACKET_MSG_ID_LOC]) {
      case UBX_VERSION_GET_MSG_ID:
        LOG_INF("GOT mon messgae");
        /* code */
        break;

      default:
        break;
    }

    return UBX_PARSER_UNKNOWN_MESSAGE_ID;
  }

  enum UBX_PARSER_STATUS process_rxm_packet() {
    DECLARE_MYL_UTILS_LOG();
    switch (rx_buffer_[UBX_PACKET_MSG_ID_LOC]) {
      case UBX_RXM_COR_MSG_ID:
        LOG_INF("GOT rxm messgae");
        /* code */
        break;
      case UBX_RXM_RTCM_MSG_ID:
        LOG_INF("GOT rtcm messgae");
        /* code */
        break;

      default:
        break;
    }

    return UBX_PARSER_UNKNOWN_MESSAGE_ID;
  }

  enum UBX_PARSER_STATUS process_rtcm_packet() {
    if (cbs_.ubx_rtcm_cb) {
      cbs_.ubx_rtcm_cb(rx_buffer_[RTCM_PACKET_PAYLOAD_ID_LOC], rx_buffer_, packet_length_);
    }
    return UBX_PARSER_PROCESSED_MSG;
  }

  void parse_nav_pvt_msg() {
    if (packet_length_ < (sizeof(struct ubx_nav_pvt) + UBX_PACKET_NON_PAYLOAD_LENGTH)) {
      return;
    }
    const struct ubx_nav_pvt* nav_pvt = (const struct ubx_nav_pvt*)(rx_buffer_ + UBX_PACKET_PAYLOAD_LOC);
    if (cbs_.ubx_nav_pvt_cb) {
      cbs_.ubx_nav_pvt_cb(*nav_pvt);
    }

    // hdop = nav_pvt->nav.pdop * 10,
    // geoid_separation = (nav_pvt->nav.height - nav_pvt->nav.hmsl),
  }

  enum UBX_PARSER_STATUS process_nav_packet() {
    switch (rx_buffer_[UBX_PACKET_MSG_ID_LOC]) {
      case UBX_NAV_PVT_MSG_ID:
        parse_nav_pvt_msg();
        return UBX_PARSER_PROCESSED_MSG;
        break;

      default:
        break;
    }

    return UBX_PARSER_UNKNOWN_MESSAGE_ID;
  }

  enum UBX_PARSER_STATUS process_full_packet() {
    uint16_t calculated_checksum = calc_checksum(rx_buffer_ + 2, packet_length_ - 4);
    DECLARE_MYL_UTILS_LOG();
    if (calculated_checksum !=
        (((uint16_t)rx_buffer_[packet_length_ - 2] << 8) + (uint16_t)rx_buffer_[packet_length_ - 1])) {
      LOG_ERR("Checksum Error");
      return UBX_PARSER_CHECKSUM_FAILURE;
    }

    switch (rx_buffer_[UBX_PACKET_CLASS_ID_LOC]) {
      case UBX_CFG:
        return process_config_packet();
        break;

      case UBX_NAV:
        return process_nav_packet();
        break;
      case UBX_MON:
        return process_mon_packet();
        break;
      case UBX_RXM:
        return process_rxm_packet();
        break;
      case UBX_RTCM:
      case UBX_INF:
      case UBX_ACK:
      case UBX_UPD:
      case UBX_TIM:
      case UBX_MGA:
      case UBX_LOG:
      case UBX_SEC:
      case UBX_NAV2:
        LOG_WRN("Got Msg %d %d\n", rx_buffer_[UBX_PACKET_CLASS_ID_LOC], rx_buffer_[UBX_PACKET_MSG_ID_LOC]);
        return UBX_PARSER_UNIMPLEMENTED_CLASS_ID;

      default:
        break;
    }
    LOG_WRN("Got Msg %d %d\n", rx_buffer_[UBX_PACKET_CLASS_ID_LOC], rx_buffer_[UBX_PACKET_MSG_ID_LOC]);

    return UBX_PARSER_UNKNOWN_CLASS_ID;
  }

  enum UBX_PARSER_STATUS ubx_parse_byte(uint8_t byte) {
    DECLARE_MYL_UTILS_LOG();
    switch (parser_state_) {
      case WAITING_FOR_SYNC1:
        if (byte == UBX_SYNC1) {
          parser_state_ = WAITING_FOR_SYNC2;
        } else if (byte == RTCM_SYNC1) {
          parser_state_ = WAITING_FOR_RTCM_LENGTH1;
        } else {
          return UBX_PARSER_SYNCING;
        }
        byte_index_ = 0;
        rx_buffer_[byte_index_] = byte;
        byte_index_++;
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
        packet_length_ = (rx_buffer_[UBX_PACKET_LENGTH_LOC] | byte << 8) + UBX_PACKET_NON_PAYLOAD_LENGTH;
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
          print_out = true;
          marker = true;
          return process_full_packet();
        }
        return UBX_PARSER_NO_MESSAGE;
        break;

      case WAITING_FOR_RTCM_LENGTH1:
        rx_buffer_[byte_index_] = byte;
        byte_index_++;
        parser_state_ = WAITING_FOR_RTCM_LENGTH2;
        return UBX_PARSER_NO_MESSAGE;
        break;

      case WAITING_FOR_RTCM_LENGTH2:
        packet_length_ = (rx_buffer_[RTCM_PACKET_LENGTH_LOC] << 8 | byte) + RTCM_PACKET_NON_PAYLOAD_LENGTH;
        if (packet_length_ > UBX_PARSER_MAX_BUFFER_SIZE) {
          parser_state_ = WAITING_FOR_SYNC1;
          return UBX_PARSER_MESSAGE_OVERSIZE;
        }
        rx_buffer_[byte_index_] = byte;
        byte_index_++;
        parser_state_ = WAITING_FOR_RTCM_PAYLOAD_AND_CHECKSUM;
        return UBX_PARSER_NO_MESSAGE;
        break;

      case WAITING_FOR_RTCM_PAYLOAD_AND_CHECKSUM:
        rx_buffer_[byte_index_] = byte;
        byte_index_++;
        if (byte_index_ == packet_length_) {
          parser_state_ = WAITING_FOR_SYNC1;
          print_out = true;
          marker = true;
          return process_rtcm_packet();
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
  ubx_callbacks cbs_;
  uint8_t rx_buffer_[1024];
  enum UBX_PARSER_STATE parser_state_ = WAITING_FOR_SYNC1;
  uint8_t byte_index_ = 0;
  uint8_t packet_length_ = 0;
  bool print_out = false;
  bool marker = false;
};