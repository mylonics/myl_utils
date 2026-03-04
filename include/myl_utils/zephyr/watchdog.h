#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/watchdog.h>

#ifndef CONFIG_MYL_UTILS_WATCHDOG
#error Enable CONFIG_MYL_UTILS_WATCHDOG to use watchdog helper
#endif

#include "log.h"

typedef void (*wdt_callback_t)(const struct device *wdt_dev, int channel_id);

struct MylWatchdog {
  const struct device *dev;
  int channel_id;
  bool started;
};

static MylWatchdog myl_wdt = {NULL, -1, false};

// Install the watchdog timeout but do NOT start it yet.
// Logs the reset cause (including watchdog resets) and clears it.
// Call myl_utils_wdt_start() after all init is complete.
static inline void myl_utils_wdt_install(uint32_t timeout_ms,
                                         wdt_callback_t callback = NULL) {
  DECLARE_MYL_UTILS_LOG();

  /* Log and clear reset cause */
  uint32_t reset_cause = 0;
  if (hwinfo_get_reset_cause(&reset_cause) == 0) {
    if (reset_cause & RESET_WATCHDOG) {
      LOG_WRN("*** Device restarted due to watchdog reset ***");
    }
    if (reset_cause & RESET_PIN) {
      LOG_INF("Reset cause: pin reset");
    }
    if (reset_cause & RESET_SOFTWARE) {
      LOG_INF("Reset cause: software reset");
    }
    if (reset_cause & RESET_POR) {
      LOG_INF("Reset cause: power-on reset");
    }
    hwinfo_clear_reset_cause();
  }

  myl_wdt = {DEVICE_DT_GET(DT_ALIAS(watchdog0)), -1, false};

  if (!device_is_ready(myl_wdt.dev)) {
    LOG_ERR("Watchdog device not ready");
    return;
  }

  struct wdt_timeout_cfg wdt_config = {};
  wdt_config.window.min = 0;
  wdt_config.window.max = timeout_ms;
  wdt_config.callback = callback;
  wdt_config.flags = WDT_FLAG_RESET_SOC;

  myl_wdt.channel_id = wdt_install_timeout(myl_wdt.dev, &wdt_config);
  if (myl_wdt.channel_id < 0) {
    LOG_ERR("Watchdog install failed: %d", myl_wdt.channel_id);
  }
}

// Start the watchdog. Call after all slow init (BLE, etc.) is complete.
static inline void myl_utils_wdt_start() {
  DECLARE_MYL_UTILS_LOG();
  if (device_is_ready(myl_wdt.dev) && myl_wdt.channel_id >= 0) {
    wdt_setup(myl_wdt.dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
    myl_wdt.started = true;
    LOG_INF("Watchdog started");
  }
}

// Feed the watchdog. Call regularly in your main loop.
static inline void myl_utils_wdt_feed() {
  if (myl_wdt.started) {
    wdt_feed(myl_wdt.dev, myl_wdt.channel_id);
  }
}
