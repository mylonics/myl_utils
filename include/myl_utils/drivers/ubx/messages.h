#pragma once

#include "stdint.h"

#define UBX_SYNC1 0xb5
#define UBX_SYNC2 0x62
#define RTCM_SYNC1 0xd3

enum UBX_CLASS_ID {
  UBX_NAV = 0x01,
  UBX_RXM = 0x02,
  UBX_INF = 0x04,
  UBX_ACK = 0x05,
  UBX_CFG = 0x06,
  UBX_UPD = 0x09,
  UBX_MON = 0x0a,
  UBX_TIM = 0x0d,
  UBX_MGA = 0x13,
  UBX_LOG = 0x21,
  UBX_SEC = 0x27,
  UBX_NAV2 = 0x29,
  UBX_RTCM = 0xf5,
};

#define MSG_ID_DEFINE(name, class_id, msg_id)              \
  const uint16_t name##_CLASS_ID = class_id << 8 | msg_id; \
  const uint8_t name##_MSG_ID = msg_id;

MSG_ID_DEFINE(UBX_NAV_PVT, UBX_NAV, 0x7);
MSG_ID_DEFINE(UBX_VERSION_GET, UBX_MON, 0x4);
MSG_ID_DEFINE(UBX_START_SURVEY, UBX_CFG, 0x71);

MSG_ID_DEFINE(UBX_RXM_COR, UBX_RXM, 0x34);
MSG_ID_DEFINE(UBX_RXM_RTCM, UBX_RXM, 0x32);

enum class UBX_CONFIG_MSG_ID : uint16_t {
  UBX_CFG_ANT = UBX_CFG << 8 | 0x13,       // Antenna Control Settings. Used to configure the antenna control settings
  UBX_CFG_BATCH = UBX_CFG << 8 | 0x93,     // Get/set data batching configuration.
  UBX_CFG_CFG = UBX_CFG << 8 | 0x09,       // Clear, Save, and Load Configurations. Used to save current configuration
  UBX_CFG_DAT = UBX_CFG << 8 | 0x06,       // Set User-defined Datum or The currently defined Datum
  UBX_CFG_DGNSS = UBX_CFG << 8 | 0x70,     // DGNSS configuration
  UBX_CFG_ESFALG = UBX_CFG << 8 | 0x56,    // ESF alignment
  UBX_CFG_ESFA = UBX_CFG << 8 | 0x4C,      // ESF accelerometer
  UBX_CFG_ESFG = UBX_CFG << 8 | 0x4D,      // ESF gyro
  UBX_CFG_GEOFENCE = UBX_CFG << 8 | 0x69,  // Geofencing configuration. Used to configure a geofence
  UBX_CFG_GNSS = UBX_CFG << 8 | 0x3E,      // GNSS system configuration
  UBX_CFG_HNR = UBX_CFG << 8 | 0x5C,       // High Navigation Rate
  UBX_CFG_INF = UBX_CFG << 8 | 0x02,   // Depending on packet length, either: poll configuration for one protocol, or
                                       // information message configuration
  UBX_CFG_ITFM = UBX_CFG << 8 | 0x39,  // Jamming/Interference Monitor configuration
  UBX_CFG_LOGFILTER = UBX_CFG << 8 | 0x47,  // Data Logger Configuration
  UBX_CFG_MSG = UBX_CFG << 8 | 0x01,        // Poll a message configuration, or Set Message Rate(s), or Set Message Rate
  UBX_CFG_NAV5 = UBX_CFG << 8 | 0x24,       // Nav. Engine Set. Used to conf. the navi engine incl. the dynamic model.
  UBX_CFG_NAVX5 = UBX_CFG << 8 | 0x23,      // Navigation Engine Expert Settings
  UBX_CFG_NMEA = UBX_CFG << 8 | 0x17,       // Extended NMEA protocol configuration V1
  UBX_CFG_ODO = UBX_CFG << 8 | 0x1E,        // Odometer, Low-speed COG Engine Settings
  UBX_CFG_PM2 = UBX_CFG << 8 | 0x3B,        // Extended power management configuration
  UBX_CFG_PMS = UBX_CFG << 8 | 0x86,        // Power mode setup
  UBX_CFG_PRT = UBX_CFG << 8 | 0x00,     // Used to configure port specifics. Polls the configuration for one I/O Port,
                                         // or Port configuration for UART ports, or Port configuration for USB port, or
                                         // Port configuration for SPI port, or Port configuration for DDC port
  UBX_CFG_PWR = UBX_CFG << 8 | 0x57,     // Put receiver in a defined power state
  UBX_CFG_RATE = UBX_CFG << 8 | 0x08,    // Navigation/Measurement Rate Settings. Used to set port baud rates.
  UBX_CFG_RINV = UBX_CFG << 8 | 0x34,    // Contents of Remote Inventory
  UBX_CFG_RST = UBX_CFG << 8 | 0x04,     // Reset Receiver / Clear Backup Data Structures. Used to reset device.
  UBX_CFG_RXM = UBX_CFG << 8 | 0x11,     // RXM configuration
  UBX_CFG_SBAS = UBX_CFG << 8 | 0x16,    // SBAS configuration
  UBX_CFG_TMODE3 = UBX_CFG << 8 | 0x71,  // Time Mode Settings 3. Used to enable Survey In Mode
  UBX_CFG_TP5 = UBX_CFG << 8 | 0x31,     // Time Pulse Parameters
  UBX_CFG_USB = UBX_CFG << 8 | 0x1B,     // USB Configuration
  // Used for config of higher version u-blox modules (ie protocol v27 and above).
  UBX_CFG_VALDEL = UBX_CFG << 8 | 0x8C,  // Deletes values corresponding to provided key-value pairs
  UBX_CFG_VALGET = UBX_CFG << 8 | 0x8B,  // Gets values corresponding to provided key-value pairs
  UBX_CFG_VALSET = UBX_CFG << 8 | 0x8A,  // Sets values corresponding to provided key-value pairs

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

  // enable RTCM Messages
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART1 = 0x209102be,  // Output rate of the RTCM-3X-TYPE1005 message on port UART1
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART2 = 0x209102bf,  // Output rate of the RTCM-3X-TYPE1005 message on port UART2
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_UART1 = 0x2091035f,  // Output rate of the RTCM-3X-TYPE1074 message on port UART1
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_UART2 = 0x20910360,  // Output rate of the RTCM-3X-TYPE1074 message on port UART2
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART1 = 0x209102cd,  // Output rate of the RTCM-3X-TYPE1077 message on port UART1
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART2 = 0x209102ce,  // Output rate of the RTCM-3X-TYPE1077 message on port UART2
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_UART1 = 0x20910364,  // Output rate of the RTCM-3X-TYPE1084 message on port UART1
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_UART2 = 0x20910365,  // Output rate of the RTCM-3X-TYPE1084 message on port UART2
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART1 = 0x209102d2,  // Output rate of the RTCM-3X-TYPE1087 message on port UART1
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART2 = 0x209102d3,  // Output rate of the RTCM-3X-TYPE1087 message on port UART2
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_UART1 = 0x20910369,  // Output rate of the RTCM-3X-TYPE1094 message on port UART1
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_UART2 = 0x2091036a,  // Output rate of the RTCM-3X-TYPE1094 message on port UART2
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART1 = 0x20910319,  // Output rate of the RTCM-3X-TYPE1097 message on port UART1
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART2 = 0x2091031a,  // Output rate of the RTCM-3X-TYPE1097 message on port UART2
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_UART1 = 0x2091036e,  // Output rate of the RTCM-3X-TYPE1124 message on port UART1
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_UART2 = 0x2091036f,  // Output rate of the RTCM-3X-TYPE1124 message on port UART2
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART1 = 0x209102d7,  // Output rate of the RTCM-3X-TYPE1127 message on port UART1
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART2 = 0x209102d8,  // Output rate of the RTCM-3X-TYPE1127 message on port UART2
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART1 = 0x20910304,  // Output rate of the RTCM-3X-TYPE1230 message on port UART1
  UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART2 = 0x20910305,  // Output rate of the RTCM-3X-TYPE1230 message on port UART2

  UBLOX_CFG_UART1INPROT_UBX = 0x10730001,      // Flag to indicate if UBX should be an input protocol on UART1
  UBLOX_CFG_UART1INPROT_RTCM3X = 0x10730004,   // Flag to indicate if RTCM3X should be an input protocol on UART1
  UBLOX_CFG_UART1OUTPROT_UBX = 0x10740001,     // Flag to indicate if UBX should be an output protocol on UART1
  UBLOX_CFG_UART1OUTPROT_RTCM3X = 0x10740004,  // Flag to indicate if RTCM3X should be an output protocol on UART1

  UBLOX_CFG_MSGOUT_UBX_RXM_RTCM_UART1 = 0x20910269,
  UBLOX_CFG_MSGOUT_UBX_RXM_COR_UART1 = 0x209106b7,
  // UBX_FRAME_DEFINE(enable_ubx_rtcm_rsp, UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_MSG_OUT_UBX_RXM_RTCM_UART1, 1));
  //  UBX_FRAME_DEFINE(set_rtk_fix_mode, UBX_FRAME_CFG_VAL_SET_U8_INITIALIZER(UBX_KEY_NAV_HP_CFG_GNSS_MODE,
  //  UBX_NAV_HP_DGNSS_MODE_RTK_FIXED));

  UBLOX_CFG_TMODE_MODE = 0x20030001,       // Receiver mode
  UBLOX_CFG_TMODE_POS_TYPE = 0x20030002,   // Determines whether the ARP position is given in ECEF or LAT/LON/HEIGHT?
  UBLOX_CFG_TMODE_ECEF_X = 0x40030003,     // ECEF X coordinate of the ARP position.
  UBLOX_CFG_TMODE_ECEF_Y = 0x40030004,     // ECEF Y coordinate of the ARP position.
  UBLOX_CFG_TMODE_ECEF_Z = 0x40030005,     // ECEF Z coordinate of the ARP position.
  UBLOX_CFG_TMODE_ECEF_X_HP = 0x20030006,  // High-precision ECEF X coordinate of the ARP position.
  UBLOX_CFG_TMODE_ECEF_Y_HP = 0x20030007,  // High-precision ECEF Y coordinate of the ARP position.
  UBLOX_CFG_TMODE_ECEF_Z_HP = 0x20030008,  // High-precision ECEF Z coordinate of the ARP position.
  UBLOX_CFG_TMODE_LAT = 0x40030009,        // Latitude of the ARP position.
  UBLOX_CFG_TMODE_LON = 0x4003000a,        // Longitude of the ARP position.
  UBLOX_CFG_TMODE_HEIGHT = 0x4003000b,     // Height of the ARP position.
  UBLOX_CFG_TMODE_LAT_HP = 0x2003000c,     // High-precision latitude of the ARP position
  UBLOX_CFG_TMODE_LON_HP = 0x2003000d,     // High-precision longitude of the ARP position.
  UBLOX_CFG_TMODE_HEIGHT_HP = 0x2003000e,  // High-precision height of the ARP position.
  UBLOX_CFG_TMODE_FIXED_POS_ACC = 0x4003000f,   // Fixed position 3D accuracy
  UBLOX_CFG_TMODE_SVIN_MIN_DUR = 0x40030010,    // Survey-in minimum duration
  UBLOX_CFG_TMODE_SVIN_ACC_LIMIT = 0x40030011,  // Survey-in position accuracy limit

};

//("CFG_MSGOUT_RTCM_3X_TYPE1005_USB", 1), ("CFG_MSGOUT_RTCM_3X_TYPE1074_USB", 1), ("CFG_MSGOUT_RTCM_3X_TYPE1077_USB",
// 1),
//     ("CFG_MSGOUT_RTCM_3X_TYPE1084_USB", 1), ("CFG_MSGOUT_RTCM_3X_TYPE1087_USB", 1),
//     ("CFG_MSGOUT_RTCM_3X_TYPE1094_USB", 1), ("CFG_MSGOUT_RTCM_3X_TYPE1097_USB", 1),
//     ("CFG_MSGOUT_RTCM_3X_TYPE1124_USB", 1), ("CFG_MSGOUT_RTCM_3X_TYPE1127_USB", 1),
//     ("CFG_MSGOUT_RTCM_3X_TYPE1230_USB", 1),

struct ubx_nav_pvt {
  struct {
    uint32_t itow;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t valid;
    uint32_t tacc;
    int32_t nano;
  } __packed time;
  uint8_t fix_type; /** See ubx_nav_fix_type */
  uint8_t flags;
  uint8_t flags2;
  struct {
    uint8_t num_sv;
    int32_t longitude;    /* Longitude. Degrees. scaling: 1e-7 */
    int32_t latitude;     /* Latitude. Degrees. scaling: 1e-7 */
    int32_t height;       /* Height above ellipsoid. mm */
    int32_t hmsl;         /* Height above mean sea level. mm */
    uint32_t horiz_acc;   /* Horizontal accuracy estimate. mm */
    uint32_t vert_acc;    /* Vertical accuracy estimate. mm */
    int32_t vel_north;    /* NED north velocity. mm/s */
    int32_t vel_east;     /* NED east velocity. mm/s */
    int32_t vel_down;     /* NED down velocity. mm/s */
    int32_t ground_speed; /* Ground Speed (2D). mm/s */
    int32_t head_motion;  /* Heading of Motion (2D). Degrees. scaling: 1e-5 */
    uint32_t speed_acc;   /* Speed accuracy estimated. mm/s */
    uint32_t head_acc;    /** Heading accuracy estimate (both motion and vehicle).
                           *  Degrees. scaling: 1e-5.
                           */
    uint16_t pdop;        /* scaling: 1e-2 */
    uint16_t flags3;
    uint32_t reserved;
    int32_t head_vehicle; /* Heading of vehicle (2D). Degrees. Valid if
                           * flags.head_vehicle_valid is set.
                           */
    int16_t mag_decl;     /* Magnetic declination. Degrees. */
    uint16_t magacc;      /* Magnetic declination accuracy. Degrees. scaling: 1e-2 */
  } __packed nav;
} __packed;

struct ubx_time_mode {
  uint8_t version;
  uint8_t reserved;
  uint8_t mode;
  uint8_t lla;
  int32_t ecefXOrLat;
  int32_t ecefYOrLong;
  int32_t ecefZOrAlt;
  int8_t ecefXOrLatHP;
  int8_t ecefYOrLonHP;
  int8_t ecefZOrAltHP;
  uint8_t reserved1;
  uint32_t fixedPosAcc;
  uint32_t svinMinDur;
  uint32_t svinAccLimit;
  uint8_t reserved2[8];
} __packed;

enum class ubx_survey_mode : uint8_t {
  DISABLED = 0,
  SURVEY_IN = 1,
  FIXED = 2,
};
