#pragma once

#include <myl_utils/zephyr/ble_helper.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

struct device_name {
  bool found;
  char name[30];
};

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (!err) {
    printk("Security changed: %s level %u\n", addr, level);
  } else {
    printk("Security failed: %s level %u err %d %s\n", addr, level, err, bt_security_err_to_str(err));
  }
}

#define NAME_LEN 30

static bool parse_data_name_cb(struct bt_data *data, void *user_data) {
  device_name *name = (device_name *)user_data;

  switch (data->type) {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
      memcpy(name->name, data->data, MIN(data->data_len, NAME_LEN - 1));
      name->found = true;
      return false;
    default:
      return true;
  }
}

static void start_scan(void) {
  int err;

  /* This demo doesn't require active scan */
  err = bt_le_scan_start(activeScan, device_found);
  if (err) {
    printk("Scanning failed to start (err %d)\n", err);
    return;
  }

  printk("Scanning successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  printk("Pairing cancelled: %s\n", addr);
}

void bt_client_init(const struct bt_conn_auth_cb *conn_auth_callbacks) {
  int err;
  if (conn_auth_callbacks) {
    err = bt_conn_auth_cb_register(conn_auth_callbacks);
    if (err) {
      printk("conn auth cb error %d \n", err);
      return;
    }
  }

  err = bt_enable(NULL);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }

  printk("Bluetooth initialized\n");

  start_scan();
}
