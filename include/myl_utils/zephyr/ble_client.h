// ---------------------------------------------------------------------------
// ble_client.h — Generic BLE Central infrastructure for multi-device
//                connections with GATT discovery, scan/connect, security
//                retry logic, and passkey authentication.
//
// Usage:
//   1. Define a BleDeviceConnection array and GattDiscoveryTarget configs.
//   2. Populate each device's name, scan prefix, target, and callbacks.
//   3. Call ble_client_init() with the device array, service UUID, and passkey.
//
// Address persistence:
//   The library stores a role→address mapping in Zephyr settings under
//   "ble_dev/<index>". On init it validates these against actual bonds
//   via bt_foreach_bond and prunes stale entries. No application-level
//   address storage is needed.
//
// Requires CONFIG_BT_SECURITY_ERR_TO_STR=y in prj.conf.
// ---------------------------------------------------------------------------
#pragma once

#include <myl_utils/zephyr/ble_helper.h>
#include <stdio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/atomic.h>

#include "log.h"

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#define MAX_TRANSIENT_RETRIES 3
#define NAME_LEN 30

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------
enum class DiscoveryAction { SUBSCRIBE_NOTIFY, WRITE_VALUE };
enum DiscoveryState { DISC_FIND_SERVICE = 0, DISC_FIND_CHAR, DISC_FIND_CCC };

struct BleDeviceConnection;

struct GattDiscoveryTarget {
  const struct bt_uuid_16 *char_uuid;
  DiscoveryAction action;
  bt_gatt_notify_func_t notify_cb;    // used when action == SUBSCRIBE_NOTIFY
  const uint8_t *write_data;          // used when action == WRITE_VALUE
  uint16_t write_len;
  void (*on_complete)(BleDeviceConnection *dev);
};

struct BleDeviceConnection {
  const char *name;
  const char *scan_name_prefix;
  size_t scan_name_prefix_len;

  struct bt_conn *conn;
  bool pairing_mode;
  uint8_t security_fail_count;

  // Paired address — loaded from settings, validated against bonds
  bt_addr_le_t paired_addr;

  // GATT discovery state
  struct bt_gatt_discover_params discover_params;
  struct bt_uuid_16 discover_uuid;
  struct bt_gatt_subscribe_params subscribe_params;
  uint16_t char_handle;
  uint16_t value_handle;
  DiscoveryState disc_state;

  const GattDiscoveryTarget *target;

  // Application-layer callbacks
  void (*on_connected_cb)(bool connected);
  void (*on_needs_pairing)();  // called when scan starts and device has no paired addr
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static bool is_credential_failure(enum bt_security_err err) {
  return (err == BT_SECURITY_ERR_AUTH_FAIL ||
          err == BT_SECURITY_ERR_PIN_OR_KEY_MISSING ||
          err == BT_SECURITY_ERR_KEY_REJECTED);
}

static bool is_valid_paired_addr(const bt_addr_le_t *addr) {
  if (bt_addr_le_eq(addr, BT_ADDR_LE_NONE)) return false;
  const uint8_t *val = addr->a.val;
  return !(val[0] == 0xFF && val[1] == 0xFF && val[2] == 0xFF &&
           val[3] == 0xFF && val[4] == 0xFF && val[5] == 0xFF);
}

// ---------------------------------------------------------------------------
// Global client state — set during ble_client_init()
// ---------------------------------------------------------------------------
static BleDeviceConnection *_ble_devices = nullptr;
static int _ble_device_count = 0;
static uint32_t _ble_passkey = 0;
static bt_security_t _ble_security_level = BT_SECURITY_L4;
static const struct bt_uuid_128 *_ble_service_uuid = nullptr;

static auto _ble_le_create_conn = BT_CONN_LE_CREATE_CONN;
static auto _ble_le_conn_param = BT_LE_CONN_PARAM_DEFAULT;
static auto _ble_active_scan = BT_LE_SCAN_ACTIVE;
static atomic_t _ble_scanning = ATOMIC_INIT(0);

// Forward declarations
static void ble_start_scan(void);
static void ble_start_scan_deferred(void);

// ---------------------------------------------------------------------------
// Address persistence — settings under "ble_dev/<device_index>"
// ---------------------------------------------------------------------------
static void ble_save_device_addr(BleDeviceConnection *dev, const bt_addr_le_t *addr) {
  DECLARE_MYL_UTILS_LOG();
  bt_addr_le_copy(&dev->paired_addr, addr);
  int idx = (int)(dev - _ble_devices);
  char key[16];
  snprintf(key, sizeof(key), "ble_dev/%d", idx);
  int err = settings_save_one(key, addr, sizeof(*addr));
  if (err) LOG_INF("Failed to save addr for %s (%d)", dev->name, err);
}

static void ble_clear_device_addr(BleDeviceConnection *dev) {
  ble_save_device_addr(dev, BT_ADDR_LE_NONE);
}

static int ble_device_settings_set(const char *name, size_t len,
                                   settings_read_cb read_cb, void *cb_arg) {
  if (!_ble_devices) return -ENOENT;
  int idx = atoi(name);
  if (idx < 0 || idx >= _ble_device_count) return -ENOENT;
  if (len != sizeof(bt_addr_le_t)) return -EINVAL;
  int rc = read_cb(cb_arg, &_ble_devices[idx].paired_addr, sizeof(bt_addr_le_t));
  return rc < 0 ? rc : 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(ble_dev, "ble_dev", NULL,
                               ble_device_settings_set, NULL, NULL);

// ---------------------------------------------------------------------------
// Bond validation — prune stored addresses with no matching bond
// ---------------------------------------------------------------------------
struct ble_bond_check {
  const bt_addr_le_t *addr;
  bool found;
};

static void ble_bond_check_cb(const struct bt_bond_info *info, void *user_data) {
  struct ble_bond_check *chk = (struct ble_bond_check *)user_data;
  if (bt_addr_le_eq(&info->addr, chk->addr)) {
    chk->found = true;
  }
}

static void ble_validate_bonds(void) {
  DECLARE_MYL_UTILS_LOG();
  for (int i = 0; i < _ble_device_count; i++) {
    BleDeviceConnection *dev = &_ble_devices[i];
    if (!is_valid_paired_addr(&dev->paired_addr)) continue;
    struct ble_bond_check chk = { &dev->paired_addr, false };
    bt_foreach_bond(BT_ID_DEFAULT, ble_bond_check_cb, &chk);
    if (!chk.found) {
      LOG_INF("%s: stale addr with no bond, clearing", dev->name);
      ble_clear_device_addr(dev);
    }
  }
}

// ---------------------------------------------------------------------------
// Device lookup
// ---------------------------------------------------------------------------
static BleDeviceConnection *ble_find_device_by_conn(struct bt_conn *conn) {
  for (int i = 0; i < _ble_device_count; i++) {
    if (_ble_devices[i].conn == conn) return &_ble_devices[i];
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// Generic GATT discovery state machine
// ---------------------------------------------------------------------------
static uint8_t ble_generic_discovery_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          struct bt_gatt_discover_params *params) {
  DECLARE_MYL_UTILS_LOG();
  BleDeviceConnection *dev = nullptr;
  for (int i = 0; i < _ble_device_count; i++) {
    if (&_ble_devices[i].discover_params == params) { dev = &_ble_devices[i]; break; }
  }
  if (!dev) return BT_GATT_ITER_STOP;

  if (!attr) {
    LOG_INF("%s: Discovery complete", dev->name);
    memset(params, 0, sizeof(*params));
    return BT_GATT_ITER_STOP;
  }

  int err;
  switch (dev->disc_state) {
    case DISC_FIND_SERVICE:
      LOG_INF("%s: Found primary service", dev->name);
      dev->discover_params.uuid = (const struct bt_uuid *)dev->target->char_uuid;
      dev->discover_params.start_handle = attr->handle + 1;
      dev->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
      dev->disc_state = DISC_FIND_CHAR;
      err = bt_gatt_discover(conn, &dev->discover_params);
      if (err) LOG_INF("%s: Discover char failed (%d)", dev->name, err);
      break;

    case DISC_FIND_CHAR:
      LOG_INF("%s: Found characteristic (handle %u)", dev->name, attr->handle);
      dev->char_handle = attr->handle;
      dev->value_handle = bt_gatt_attr_value_handle(attr);

      if (dev->target->action == DiscoveryAction::SUBSCRIBE_NOTIFY) {
        memcpy(&dev->discover_uuid, BT_UUID_GATT_CCC, sizeof(dev->discover_uuid));
        dev->discover_params.uuid = &dev->discover_uuid.uuid;
        dev->discover_params.start_handle = attr->handle + 2;
        dev->discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
        dev->subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
        dev->disc_state = DISC_FIND_CCC;
        err = bt_gatt_discover(conn, &dev->discover_params);
        if (err) LOG_INF("%s: Discover CCC failed (%d)", dev->name, err);
      } else {
        // WRITE_VALUE — send initial data to the characteristic
        if (dev->target->write_data && dev->target->write_len > 0) {
          err = bt_gatt_write_without_response(conn, dev->value_handle,
                                               dev->target->write_data,
                                               dev->target->write_len, false);
          if (err) LOG_INF("%s: Write failed (%d)", dev->name, err);
          else LOG_INF("%s: Value written", dev->name);
        }
        if (dev->target->on_complete) dev->target->on_complete(dev);
      }
      break;

    case DISC_FIND_CCC:
      LOG_INF("%s: Found CCC descriptor", dev->name);
      dev->subscribe_params.notify = dev->target->notify_cb;
      dev->subscribe_params.value = BT_GATT_CCC_NOTIFY;
      dev->subscribe_params.ccc_handle = attr->handle;
      err = bt_gatt_subscribe(conn, &dev->subscribe_params);
      if (err && err != -EALREADY) {
        LOG_INF("%s: Subscribe failed (%d)", dev->name, err);
      } else {
        LOG_INF("%s: [SUBSCRIBED]", dev->name);
        if (dev->target->on_complete) dev->target->on_complete(dev);
      }
      break;
  }

  return BT_GATT_ITER_STOP;
}

static void ble_start_gatt_discovery(struct bt_conn *conn) {
  DECLARE_MYL_UTILS_LOG();
  BleDeviceConnection *dev = ble_find_device_by_conn(conn);
  if (!dev || !dev->target || !_ble_service_uuid) return;

  dev->discover_params.uuid = &_ble_service_uuid->uuid;
  dev->discover_params.func = ble_generic_discovery_func;
  dev->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
  dev->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
  dev->discover_params.type = BT_GATT_DISCOVER_PRIMARY;
  dev->disc_state = DISC_FIND_SERVICE;

  int err = bt_gatt_discover(conn, &dev->discover_params);
  if (err) LOG_INF("%s: Discovery failed (%d)", dev->name, err);
}

// ---------------------------------------------------------------------------
// Scan / connect infrastructure
// ---------------------------------------------------------------------------
struct device_name {
  bool found;
  char name[NAME_LEN];
};

static bool ble_data_cb(struct bt_data *data, void *user_data) {
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

static void ble_create_connection(struct bt_conn **conn, const bt_addr_le_t *addr,
                                  char *addr_str, const char *dev_name) {
  DECLARE_MYL_UTILS_LOG();
  if (bt_le_scan_stop()) return;
  atomic_clear(&_ble_scanning);

  int err = bt_conn_le_create(addr, _ble_le_create_conn, _ble_le_conn_param, conn);
  if (err) {
    LOG_INF("Create %s conn to %s failed (%d)", dev_name, addr_str, err);
    ble_start_scan_deferred();
  } else {
    LOG_INF("Create %s conn to %s succeeded", dev_name, addr_str);
  }
}

// ---------------------------------------------------------------------------
// Scan callback — generic device matching
// ---------------------------------------------------------------------------
static void ble_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                             struct net_buf_simple *ad) {
  DECLARE_MYL_UTILS_LOG();
  // Check if all devices are already connected
  bool all_connected = true;
  for (int i = 0; i < _ble_device_count; i++) {
    if (!_ble_devices[i].conn) { all_connected = false; break; }
  }
  if (all_connected) return;

  if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND &&
      type != BT_GAP_ADV_TYPE_SCAN_RSP) {
    return;
  }

  char addr_str[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

  // Priority 1: Reconnect to a paired device by address
  // NOTE: This direct address comparison requires CONFIG_BT_PRIVACY=n on
  // peripherals.  If privacy is ever enabled the peripheral's address will
  // rotate via IRK and bt_addr_le_is_bonded() should be used instead.
  for (int i = 0; i < _ble_device_count; i++) {
    BleDeviceConnection *dev = &_ble_devices[i];
    if (dev->conn) continue;
    if (is_valid_paired_addr(&dev->paired_addr) && bt_addr_le_eq(&dev->paired_addr, addr)) {
      LOG_INF("Reconnecting to paired %s", dev->name);
      ble_create_connection(&dev->conn, addr, addr_str, dev->name);
      return;
    }
  }

  // Priority 2: Pair a new device by advertised name
  device_name name{};
  bt_data_parse(ad, ble_data_cb, &name);
  if (!name.found) return;

  for (int i = 0; i < _ble_device_count; i++) {
    BleDeviceConnection *dev = &_ble_devices[i];
    if (dev->conn) continue;
    if (is_valid_paired_addr(&dev->paired_addr)) continue;
    if (!dev->pairing_mode) continue;
    if (!strncmp(dev->scan_name_prefix, name.name, dev->scan_name_prefix_len)) {
      LOG_INF("Found new %s: %s", dev->name, addr_str);
      ble_create_connection(&dev->conn, addr, addr_str, dev->name);
      return;
    }
  }
}

// ---------------------------------------------------------------------------
// Deferred scan restart
// ---------------------------------------------------------------------------
static void ble_scan_work_handler(struct k_work *work) {
  (void)work;
  ble_start_scan();
}

K_WORK_DELAYABLE_DEFINE(ble_scan_work, ble_scan_work_handler);

static void ble_start_scan_deferred(void) {
  DECLARE_MYL_UTILS_LOG();
  LOG_INF("Deferring scan restart");
  k_work_reschedule(&ble_scan_work, K_MSEC(200));
}

static void ble_start_scan(void) {
  DECLARE_MYL_UTILS_LOG();
  if (atomic_get(&_ble_scanning)) {
    LOG_INF("Scan already active");
    return;
  }

  int err = bt_le_scan_start(_ble_active_scan, ble_device_found);
  if (err) {
    LOG_INF("Scanning failed to start (%d)", err);
    return;
  }

  LOG_INF("Scanning started");
  atomic_set(&_ble_scanning, 1);

  // Notify devices that need pairing attention
  for (int i = 0; i < _ble_device_count; i++) {
    BleDeviceConnection *dev = &_ble_devices[i];
    if (dev->on_needs_pairing) {
      if (!is_valid_paired_addr(&dev->paired_addr) && !dev->pairing_mode) {
        dev->on_needs_pairing();
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Connection callbacks
// ---------------------------------------------------------------------------
static void ble_security_changed(struct bt_conn *conn, bt_security_t level,
                                 enum bt_security_err err) {
  DECLARE_MYL_UTILS_LOG();
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  BleDeviceConnection *dev = ble_find_device_by_conn(conn);

  if (!err) {
    LOG_INF("Security changed: %s level %u", addr, level);

    if (dev) {
      // Address is saved in ble_pairing_complete once bond is persisted
      dev->pairing_mode = false;
      dev->security_fail_count = 0;
    }

    ble_start_gatt_discovery(conn);

    // Continue scanning for remaining devices
    for (int i = 0; i < _ble_device_count; i++) {
      if (!_ble_devices[i].conn) { ble_start_scan(); break; }
    }
  } else {
    LOG_INF("Security failed: %s level %u err %d %s", addr, level, err,
            bt_security_err_to_str(err));

    if (dev) {
      dev->security_fail_count++;

      if (is_credential_failure(err)) {
        LOG_INF("%s credential failure (err %d), clearing bond and address",
                dev->name, err);
        bt_unpair(BT_ID_DEFAULT, bt_conn_get_dst(conn));
        ble_clear_device_addr(dev);
        dev->security_fail_count = 0;
        dev->pairing_mode = true;
      } else {
        // Transient failure — disconnect and retry, but never unpair.
        // Unpairing here would desync from the server (which still holds
        // the bond), creating a deadlock where neither side can reconnect
        // without manual intervention.
        LOG_INF("%s transient failure (err %d), retry %d",
                dev->name, err, dev->security_fail_count);
      }
    }

    bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
    ble_start_scan_deferred();
  }
}

static void ble_connected(struct bt_conn *conn, uint8_t err) {
  DECLARE_MYL_UTILS_LOG();
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (err) {
    LOG_INF("Failed to connect to %s %u %s", addr, err, bt_hci_err_to_str(err));
    BleDeviceConnection *dev = ble_find_device_by_conn(conn);
    if (dev) {
      bt_conn_unref(dev->conn);
      dev->conn = NULL;
    }
    ble_start_scan_deferred();
    return;
  }

  LOG_INF("Connected: %s", addr);
  atomic_clear(&_ble_scanning);

  err = bt_conn_set_security(conn, _ble_security_level);
  if (err) LOG_INF("Failed to set security (%d)", err);
}

static void ble_disconnected(struct bt_conn *conn, uint8_t reason) {
  DECLARE_MYL_UTILS_LOG();
  BleDeviceConnection *dev = ble_find_device_by_conn(conn);
  if (!dev) return;

  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

  bt_conn_unref(dev->conn);
  dev->conn = NULL;
  dev->char_handle = 0;
  dev->value_handle = 0;

  if (dev->on_connected_cb) dev->on_connected_cb(false);

  if (!atomic_get(&_ble_scanning)) ble_start_scan_deferred();
}

BT_CONN_CB_DEFINE(ble_client_conn_callbacks) = {
    .connected = ble_connected,
    .disconnected = ble_disconnected,
    .security_changed = ble_security_changed,
};

// ---------------------------------------------------------------------------
// Authentication callbacks
// ---------------------------------------------------------------------------
static void ble_auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
  DECLARE_MYL_UTILS_LOG();
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void ble_auth_cancel(struct bt_conn *conn) {
  DECLARE_MYL_UTILS_LOG();
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Pairing cancelled: %s", addr);
}

static void ble_passkey_entry(struct bt_conn *conn) {
  DECLARE_MYL_UTILS_LOG();
  BleDeviceConnection *dev = ble_find_device_by_conn(conn);
  LOG_INF("Passkey entry for %s", dev ? dev->name : "UNKNOWN");
  int err = bt_conn_auth_passkey_entry(conn, _ble_passkey);
  if (err) LOG_INF("Passkey Entry Error %d", err);
}

static struct bt_conn_auth_cb ble_conn_auth_callbacks = {
    .passkey_display = ble_auth_passkey_display,
    .passkey_entry = ble_passkey_entry,
    .cancel = ble_auth_cancel,
};

// ---------------------------------------------------------------------------
// Auth info callbacks — save address only after bond is fully persisted
// ---------------------------------------------------------------------------
static void ble_pairing_complete(struct bt_conn *conn, bool bonded) {
  DECLARE_MYL_UTILS_LOG();
  BleDeviceConnection *dev = ble_find_device_by_conn(conn);
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (dev && bonded) {
    ble_save_device_addr(dev, bt_conn_get_dst(conn));
    LOG_INF("%s: pairing complete, bond stored for %s", dev->name, addr);
  } else if (dev) {
    LOG_INF("%s: pairing complete (not bonded)", dev->name);
  }
}

static void ble_pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
  DECLARE_MYL_UTILS_LOG();
  BleDeviceConnection *dev = ble_find_device_by_conn(conn);
  LOG_ERR("%s: pairing failed (reason %d)",
          dev ? dev->name : "UNKNOWN", reason);
}

static struct bt_conn_auth_info_cb ble_conn_auth_info_callbacks = {
    .pairing_complete = ble_pairing_complete,
    .pairing_failed = ble_pairing_failed,
};

// ---------------------------------------------------------------------------
// Initialization & utility
// ---------------------------------------------------------------------------
void ble_client_init(BleDeviceConnection *devices, int device_count,
                     const struct bt_uuid_128 *service_uuid,
                     uint32_t passkey,
                     bt_security_t security_level = BT_SECURITY_L4) {
  DECLARE_MYL_UTILS_LOG();
  _ble_devices = devices;
  _ble_device_count = device_count;
  _ble_service_uuid = service_uuid;
  _ble_passkey = passkey;
  _ble_security_level = security_level;

  int err = bt_conn_auth_cb_register(&ble_conn_auth_callbacks);
  if (err) { LOG_ERR("conn auth cb error %d", err); return; }

  err = bt_conn_auth_info_cb_register(&ble_conn_auth_info_callbacks);
  if (err) { LOG_ERR("auth info cb error %d", err); return; }

  err = bt_enable(NULL);
  if (err) { LOG_ERR("Bluetooth init failed (%d)", err); return; }

  LOG_INF("Bluetooth initialized");
  settings_load();
  ble_validate_bonds();
  ble_start_scan();
}

static void ble_disconnect_device(BleDeviceConnection *dev) {
  // Disconnect first, then unpair — calling bt_unpair on an active
  // connection can trigger the disconnected callback synchronously,
  // leading to double-disconnect and scan restart races.
  if (dev->conn) {
    bt_conn_disconnect(dev->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
  }
  if (is_valid_paired_addr(&dev->paired_addr)) {
    bt_unpair(BT_ID_DEFAULT, &dev->paired_addr);
  }
  ble_clear_device_addr(dev);
}
