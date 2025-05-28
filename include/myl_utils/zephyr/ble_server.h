
#pragma once

#include <errno.h>
#include <myl_utils/zephyr/ble_helper.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

#include "myl_utils/zephyr/hw_helper.h"

BLE_DEFAULT_AD_SD()

static struct k_work adv_work;
static struct k_work pairing_work;

static bool isPairingMode(bool set = false, bool value = false) {
  static bool pairing_mode = false;
  if (set) {
    pairing_mode = value;
  }
  return pairing_mode;
};

static void pairing_work_handler(struct k_work *work) {
  int err = bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
  DECLARE_MYL_UTILS_LOG();
  if (err) {
    LOG_INF("Cannot delete bond (err: %d)\n", err);
  } else {
    LOG_INF("Bond deleted succesfully");
  }

  isPairingMode(true, true);
  int err_code = bt_le_adv_stop();
  if (err_code) {
    DECLARE_MYL_UTILS_LOG();
    LOG_INF("Cannot stop advertising err= %d \n", err_code);
    return;
  }
}

static void setup_accept_list_cb(const struct bt_bond_info *info, void *user_data) {
  int *bond_cnt = (int *)user_data;
  if ((*bond_cnt) < 0) {
    return;
  }
  int err = bt_le_filter_accept_list_add(&info->addr);
  DECLARE_MYL_UTILS_LOG();
  LOG_INF("Added following peer to whitelist: %x %x \n", info->addr.a.val[0], info->addr.a.val[1]);
  if (err) {
    LOG_INF("Cannot add peer to Filter Accept List (err: %d)\n", err);
    (*bond_cnt) = -EIO;
  } else {
    (*bond_cnt)++;
  }
}

static int setup_accept_list(uint8_t local_id) {
  DECLARE_MYL_UTILS_LOG();
  LOG_INF("In Setup Accept List function");
  int err = bt_le_filter_accept_list_clear();
  if (err) {
    LOG_INF("Cannot clear Filter Accept List (err: %d)\n", err);
    return err;
  }
  int bond_cnt = 0;
  bt_foreach_bond(local_id, setup_accept_list_cb, &bond_cnt);
  return bond_cnt;
}

static void adv_work_handler(struct k_work *work) {
  DECLARE_MYL_UTILS_LOG();
  int err = 0;
  if (isPairingMode()) {
    err = bt_le_filter_accept_list_clear();
    if (err) {
      LOG_INF("Cannot clear accept list (err: %d)\n", err);
    } else {
      LOG_INF("Accept list cleared succesfully");
    }
    isPairingMode(true, false);
    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
      LOG_INF("Advertising failed to start (err %d)\n", err);
      return;
    }
    LOG_INF("Advertising successfully started\n");
    return;
  }

  int allowed_cnt = setup_accept_list(BT_ID_DEFAULT);
  if (allowed_cnt < 0) {
    LOG_INF("Acceptlist setup failed (err:%d)\n", allowed_cnt);
  } else {
    if (allowed_cnt == 0) {
      LOG_INF("Advertising with no Accept list \n");
      err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    } else {
      LOG_INF("Advertising with Accept list \n");
      LOG_INF("Acceptlist setup number  = %d \n", allowed_cnt);
      err = bt_le_adv_start(BT_LE_ADV_CONN_ACCEPT_LIST, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    }
    if (err) {
      LOG_INF("Advertising failed to start (err %d)\n", err);
      return;
    }
    LOG_INF("Advertising successfully started\n");
  }
}

static void advertising_start(void) { k_work_submit(&adv_work); }

void pairing_start(void) { k_work_submit(&pairing_work); }

static void connected(struct bt_conn *conn, uint8_t err) {
  DECLARE_MYL_UTILS_LOG();
  if (err) {
    LOG_INF("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
  } else {
    LOG_INF("Connected\n");
  }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
  DECLARE_MYL_UTILS_LOG();
  LOG_INF("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
}

static void recycled() { advertising_start(); }

static void on_security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  DECLARE_MYL_UTILS_LOG();

  if (!err) {
    LOG_INF("Security changed: %s level %u\n", addr, level);
  } else {
    LOG_INF("Security failed: %s level %u err %d\n", addr, level, err);
  }
}

/* STEP 9.1 - Define the callback function auth_passkey_display */
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  DECLARE_MYL_UTILS_LOG();

  LOG_INF("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  DECLARE_MYL_UTILS_LOG();

  LOG_INF("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_display = auth_passkey_display,
    .cancel = auth_cancel,
};

BT_CONN_CB_DEFINE(conn_callbacks) = {.connected = connected,
                                     .disconnected = disconnected,
                                     .recycled = recycled,
                                     .security_changed = on_security_changed};

void bt_ready(void) {
  DECLARE_MYL_UTILS_LOG();
  LOG_INF("Bluetooth initialized\n");

  settings_load();

  k_work_init(&adv_work, adv_work_handler);
  k_work_init(&pairing_work, pairing_work_handler);
  advertising_start();
}

int initialize_basic_ble(int passkey) {
  DECLARE_MYL_UTILS_LOG();
  if (passkey) {
    bt_passkey_set(191919);
  }
  int err = bt_conn_auth_cb_register(&conn_auth_callbacks);
  if (err) {
    LOG_INF("Failed to register authorization callbacks\n");
    return -1;
  }

  err = bt_enable(NULL);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return err;
  }

  bt_ready();
  return 0;
}