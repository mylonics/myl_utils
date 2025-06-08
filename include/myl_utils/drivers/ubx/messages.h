#pragma once

#include "stdint.h"

#define UBX_SYNC1 0xb5
#define UBX_SYNC2 0x62

enum UBX_CLASS_ID {
  UBX_NAV = 0x01,
  UBX_RXM = 0x02,
  UBX_INF = 0x04,
  UBX_ACK = 0x05,
  UBX_CONFIG = 0x06,
  UBX_UPD = 0x09,
  UBX_MON = 0x0a,
  UBX_TIM = 0x0d,
  UBX_MGA = 0x13,
  UBX_LOG = 0x21,
  UBX_SEC = 0x27,
  UBX_NAV2 = 0x29,
};

enum class UBX_MON_MSG_ID : uint16_t {
  UBX_VERSION_GET = UBX_MON << 8 | 0x4,
};

enum class UBX_CONFIG_MSG_ID : uint16_t {
  UBX_CFG_ANT = UBX_CONFIG << 8 | 0x13,     // Antenna Control Settings. Used to configure the antenna control settings
  UBX_CFG_BATCH = UBX_CONFIG << 8 | 0x93,   // Get/set data batching configuration.
  UBX_CFG_CFG = UBX_CONFIG << 8 | 0x09,     // Clear, Save, and Load Configurations. Used to save current configuration
  UBX_CFG_DAT = UBX_CONFIG << 8 | 0x06,     // Set User-defined Datum or The currently defined Datum
  UBX_CFG_DGNSS = UBX_CONFIG << 8 | 0x70,   // DGNSS configuration
  UBX_CFG_ESFALG = UBX_CONFIG << 8 | 0x56,  // ESF alignment
  UBX_CFG_ESFA = UBX_CONFIG << 8 | 0x4C,    // ESF accelerometer
  UBX_CFG_ESFG = UBX_CONFIG << 8 | 0x4D,    // ESF gyro
  UBX_CFG_GEOFENCE = UBX_CONFIG << 8 | 0x69,  // Geofencing configuration. Used to configure a geofence
  UBX_CFG_GNSS = UBX_CONFIG << 8 | 0x3E,      // GNSS system configuration
  UBX_CFG_HNR = UBX_CONFIG << 8 | 0x5C,       // High Navigation Rate
  UBX_CFG_INF = UBX_CONFIG << 8 | 0x02,   // Depending on packet length, either: poll configuration for one protocol, or
                                          // information message configuration
  UBX_CFG_ITFM = UBX_CONFIG << 8 | 0x39,  // Jamming/Interference Monitor configuration
  UBX_CFG_LOGFILTER = UBX_CONFIG << 8 | 0x47,  // Data Logger Configuration
  UBX_CFG_MSG = UBX_CONFIG << 8 | 0x01,    // Poll a message configuration, or Set Message Rate(s), or Set Message Rate
  UBX_CFG_NAV5 = UBX_CONFIG << 8 | 0x24,   // Nav. Engine Set. Used to conf. the navi engine incl. the dynamic model.
  UBX_CFG_NAVX5 = UBX_CONFIG << 8 | 0x23,  // Navigation Engine Expert Settings
  UBX_CFG_NMEA = UBX_CONFIG << 8 | 0x17,   // Extended NMEA protocol configuration V1
  UBX_CFG_ODO = UBX_CONFIG << 8 | 0x1E,    // Odometer, Low-speed COG Engine Settings
  UBX_CFG_PM2 = UBX_CONFIG << 8 | 0x3B,    // Extended power management configuration
  UBX_CFG_PMS = UBX_CONFIG << 8 | 0x86,    // Power mode setup
  UBX_CFG_PRT = UBX_CONFIG << 8 | 0x00,  // Used to configure port specifics. Polls the configuration for one I/O Port,
                                         // or Port configuration for UART ports, or Port configuration for USB port, or
                                         // Port configuration for SPI port, or Port configuration for DDC port
  UBX_CFG_PWR = UBX_CONFIG << 8 | 0x57,  // Put receiver in a defined power state
  UBX_CFG_RATE = UBX_CONFIG << 8 | 0x08,    // Navigation/Measurement Rate Settings. Used to set port baud rates.
  UBX_CFG_RINV = UBX_CONFIG << 8 | 0x34,    // Contents of Remote Inventory
  UBX_CFG_RST = UBX_CONFIG << 8 | 0x04,     // Reset Receiver / Clear Backup Data Structures. Used to reset device.
  UBX_CFG_RXM = UBX_CONFIG << 8 | 0x11,     // RXM configuration
  UBX_CFG_SBAS = UBX_CONFIG << 8 | 0x16,    // SBAS configuration
  UBX_CFG_TMODE3 = UBX_CONFIG << 8 | 0x71,  // Time Mode Settings 3. Used to enable Survey In Mode
  UBX_CFG_TP5 = UBX_CONFIG << 8 | 0x31,     // Time Pulse Parameters
  UBX_CFG_USB = UBX_CONFIG << 8 | 0x1B,     // USB Configuration
  // Used for config of higher version u-blox modules (ie protocol v27 and above).
  UBX_CFG_VALDEL = UBX_CONFIG << 8 | 0x8C,  // Deletes values corresponding to provided key-value pairs
  UBX_CFG_VALGET = UBX_CONFIG << 8 | 0x8B,  // Gets values corresponding to provided key-value pairs
  UBX_CFG_VALSET = UBX_CONFIG << 8 | 0x8A,  // Sets values corresponding to provided key-value pairs
};

enum class ubx_fix_mode : uint8_t {
  UBX_FIX_MODE_2D_ONLY = 1,
  UBX_FIX_MODE_3D_ONLY = 2,
  UBX_FIX_MODE_AUTO = 3,
};

enum ubx_keys_msg_out {
  UBX_KEY_MSG_OUT_NMEA_GGA_UART1 = 0x209100bb,
  UBX_KEY_MSG_OUT_NMEA_RMC_UART1 = 0x209100ac,
  UBX_KEY_MSG_OUT_NMEA_GSV_UART1 = 0x209100c5,
  UBX_KEY_MSG_OUT_NMEA_DTM_UART1 = 0x209100a7,
  UBX_KEY_MSG_OUT_NMEA_GBS_UART1 = 0x209100de,
  UBX_KEY_MSG_OUT_NMEA_GLL_UART1 = 0x209100ca,
  UBX_KEY_MSG_OUT_NMEA_GNS_UART1 = 0x209100b6,
  UBX_KEY_MSG_OUT_NMEA_GRS_UART1 = 0x209100cf,
  UBX_KEY_MSG_OUT_NMEA_GSA_UART1 = 0x209100c0,
  UBX_KEY_MSG_OUT_NMEA_GST_UART1 = 0x209100d4,
  UBX_KEY_MSG_OUT_NMEA_VTG_UART1 = 0x209100b1,
  UBX_KEY_MSG_OUT_NMEA_VLW_UART1 = 0x209100e8,
  UBX_KEY_MSG_OUT_NMEA_ZDA_UART1 = 0x209100d9,
  UBX_KEY_MSG_OUT_UBX_NAV_PVT_UART1 = 0x20910007,
  UBX_KEY_MSG_OUT_UBX_NAV_SAT_UART1 = 0x20910016,

  UBLOX_CFG_HW_ANT_CFG_VOLTCTRL = 0x10a3002e,      // Active antenna voltage control flag
  UBLOX_CFG_HW_ANT_CFG_SHORTDET = 0x10a3002f,      // Short antenna detection flag
  UBLOX_CFG_HW_ANT_CFG_SHORTDET_POL = 0x10a30030,  // Short antenna detection polarity
  UBLOX_CFG_HW_ANT_CFG_OPENDET = 0x10a30031,       // Open antenna detection flag
  UBLOX_CFG_HW_ANT_CFG_OPENDET_POL = 0x10a30032,   // Open antenna detection polarity
  UBLOX_CFG_HW_ANT_CFG_PWRDOWN = 0x10a30033,       // Power down antenna flag
  UBLOX_CFG_HW_ANT_CFG_PWRDOWN_POL = 0x10a30034,   // Power down antenna logic polarity
  UBLOX_CFG_HW_ANT_CFG_RECOVER = 0x10a30035,       // Automatic recovery from short state flag
  UBLOX_CFG_HW_ANT_SUP_SWITCH_PIN = 0x20a30036,    // ANT1 PIO number
  UBLOX_CFG_HW_ANT_SUP_SHORT_PIN = 0x20a30037,     // ANT0 PIO number
  UBLOX_CFG_HW_ANT_SUP_OPEN_PIN = 0x20a30038,      // ANT2 PIO number
  UBLOX_CFG_HW_ANT_SUP_ENGINE = 0x20a30054,        // Antenna supervisor engine selection
  UBLOX_CFG_HW_ANT_SUP_SHORT_THR = 0x20a30055,     // Antenna supervisor MADC engine short detection threshold
  UBLOX_CFG_HW_ANT_SUP_OPEN_THR = 0x20a30056,      // Antenna supervisor MADC engine open detection threshold

  UBX_KEY_NAV_CFG_FIX_MODE = 0x20110011,
  UBX_KEY_NAV_CFG_DYN_MODEL = 0x20110021,
};
