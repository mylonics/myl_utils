#pragma once

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#define BLE_SERVIE_UUID_CREATE(name, w32, w1, w2, w3, w48) BT_UUID_128_ENCODE(w32, w1, w2, w3, w48)

#define BLE_SERVIE_DECLARE(name) BT_UUID_DECLARE_128(BT_UUID_##name##_SERVICE_VAL)

#define BLE_SERVIE_DEFINE(name, w32, w1, w2, w3, w48) \
  BLE_SERVIE_UUID_CREATE(name, w32, w1, w2, w3, w48)  \
  BLE_SERVIE_DECLARE(name)

#define BLE_CHARACTERISTIC_UUID(name, value) \
  const struct bt_uuid_16 name##_CHAR = {    \
      .uuid = {BT_UUID_TYPE_16},             \
      .val = (value),                        \
  };

#define BLE_SERVICE_UUID(name, w32, w1, w2, w3, w48) \
  const struct bt_uuid_128 name##_SERVICE_UUID = BT_UUID_INIT_128(BT_UUID_128_ENCODE(w32, w1, w2, w3, w48));

#define BLE_R_CHARAC_DECLARATION(name, read_callback)                                                               \
  BT_GATT_CHARACTERISTIC((const struct bt_uuid *)&name##_CHAR, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_callback, \
                         NULL, NULL),                                                                               \
      BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)

#define BLE_RN_CHARAC_DECLARATION(name, read_callback)                                                  \
  BT_GATT_CHARACTERISTIC((const struct bt_uuid *)&name##_CHAR, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, \
                         BT_GATT_PERM_READ, read_callback, NULL, NULL),                                 \
      BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)

#define BLE_RN_ENCRYPT_CHARAC_DECLARATION(name, read_callback)                                          \
  BT_GATT_CHARACTERISTIC((const struct bt_uuid *)&name##_CHAR, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, \
                         BT_GATT_PERM_READ_ENCRYPT, read_callback, NULL, NULL),                         \
      BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)

#define BLE_RW_CHARAC_DECLARATION(name, read_callback, write_callback)                                 \
  BT_GATT_CHARACTERISTIC((const struct bt_uuid *)&name##_CHAR, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, \
                         BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_callback, write_callback, NULL), \
      BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)

#define BLE_RW_ENCRYPT_CHARAC_DECLARATION(name, read_callback, write_callback)                                         \
  BT_GATT_CHARACTERISTIC((const struct bt_uuid *)&name##_CHAR, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,                 \
                         BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_callback, write_callback, NULL), \
      BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)

#define BLE_DEFAULT_AD_SD()                                                                                  \
  static const struct bt_data ad[] = {BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR))}; \
  static const struct bt_data sd[] = {                                                                       \
      BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),              \
  };

#define BT_LE_ADV_CONN_ACCEPT_LIST                                                           \
  BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN, BT_GAP_ADV_FAST_INT_MIN_2, \
                  BT_GAP_ADV_FAST_INT_MAX_2, NULL)
