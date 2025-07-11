/*
 * Copyright (c) 2023 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef CONFIG_MYL_UTILS_USB_SERIAL

#include <myl_utils/zephyr/log.h>
#include <stdint.h>
#include <zephyr/usb/usbd.h>

/*
 * The scope of this header is limited to use in USB samples together with the
 * new experimental USB device stack, you should not use it in your own
 * application. However, you can use the code as a template.
 */

/*
 * This function uses Kconfig.sample_usbd options to configure and initialize a
 * USB device. It configures sample's device context, default string
 * descriptors, USB device configuration, registers any available class
 * instances, and finally initializes USB device. It is limited to a single
 * device with a single configuration instantiated in sample_usbd_init.c, which
 * should be enough for a simple USB device sample.
 *
 * It returns the configured and initialized USB device context on success,
 * otherwise it returns NULL.
 */

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/bos.h>
#include <zephyr/usb/usbd.h>

#define ZEPHYR_PROJECT_USB_VID 0x2fe3

/*
 * This does not yet provide valuable information, but rather serves as an
 * example, and will be improved in the future.
 */
static const struct usb_bos_capability_lpm bos_cap_lpm = {
    .bLength = sizeof(struct usb_bos_capability_lpm),
    .bDescriptorType = USB_DESC_DEVICE_CAPABILITY,
    .bDevCapabilityType = USB_BOS_CAPABILITY_EXTENSION,
    .bmAttributes = 0UL,
};

static void sample_fix_code_triple(struct usbd_context *uds_ctx, const enum usbd_speed speed) {
  /* Always use class code information from Interface Descriptors */
  if (IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS) || IS_ENABLED(CONFIG_USBD_CDC_ECM_CLASS) ||
      IS_ENABLED(CONFIG_USBD_AUDIO2_CLASS)) {
    /*
     * Class with multiple interfaces have an Interface
     * Association Descriptor available, use an appropriate triple
     * to indicate it.
     */
    usbd_device_set_code_triple(uds_ctx, speed, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
  } else {
    usbd_device_set_code_triple(uds_ctx, speed, 0, 0, 0);
  }
}

struct usbd_context *sample_usbd_init_device(usbd_msg_cb_t msg_cb) {
  DECLARE_MYL_UTILS_LOG()
  /* doc device instantiation start */
  /*
   * Instantiate a context named sample_usbd using the default USB device
   * controller, the Zephyr project vendor ID, and the sample product ID.
   * Zephyr project vendor ID must not be used outside of Zephyr samples.
   */
  USBD_DEVICE_DEFINE(sample_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), ZEPHYR_PROJECT_USB_VID,
                     CONFIG_MYL_UTILS_USBD_PID);
  /* doc device instantiation end */

  /* doc string instantiation start */
  USBD_DESC_LANG_DEFINE(sample_lang);
  USBD_DESC_MANUFACTURER_DEFINE(sample_mfr, CONFIG_MYL_UTILS_USBD_MANUFACTURER);
  USBD_DESC_PRODUCT_DEFINE(sample_product, CONFIG_MYL_UTILS_USBD_PRODUCT);
  USBD_DESC_SERIAL_NUMBER_DEFINE(sample_sn);
  /* doc string instantiation end */

  USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "FS Configuration");
  USBD_DESC_CONFIG_DEFINE(hs_cfg_desc, "HS Configuration");

  /* doc configuration instantiation start */
  static const uint8_t attributes = (IS_ENABLED(CONFIG_MYL_UTILS_USBD_SELF_POWERED) ? USB_SCD_SELF_POWERED : 0) |
                                    (IS_ENABLED(CONFIG_MYL_UTILS_USBD_REMOTE_WAKEUP) ? USB_SCD_REMOTE_WAKEUP : 0);

  /* Full speed configuration */
  USBD_CONFIGURATION_DEFINE(sample_fs_config, attributes, CONFIG_MYL_UTILS_USBD_MAX_POWER, &fs_cfg_desc);

  /* High speed configuration */
  USBD_CONFIGURATION_DEFINE(sample_hs_config, attributes, CONFIG_MYL_UTILS_USBD_MAX_POWER, &hs_cfg_desc);
  /* doc configuration instantiation end */

  USBD_DESC_BOS_DEFINE(sample_usbext, sizeof(bos_cap_lpm), &bos_cap_lpm);

  int err;

  /* doc add string descriptor start */
  err = usbd_add_descriptor(&sample_usbd, &sample_lang);
  if (err) {
    LOG_ERR("Failed to initialize language descriptor (%d)", err);
    return NULL;
  }

  err = usbd_add_descriptor(&sample_usbd, &sample_mfr);
  if (err) {
    LOG_ERR("Failed to initialize manufacturer descriptor (%d)", err);
    return NULL;
  }

  err = usbd_add_descriptor(&sample_usbd, &sample_product);
  if (err) {
    LOG_ERR("Failed to initialize product descriptor (%d)", err);
    return NULL;
  }

  err = usbd_add_descriptor(&sample_usbd, &sample_sn);
  if (err) {
    LOG_ERR("Failed to initialize SN descriptor (%d)", err);
    return NULL;
  }
  /* doc add string descriptor end */

  if (usbd_caps_speed(&sample_usbd) == USBD_SPEED_HS) {
    err = usbd_add_configuration(&sample_usbd, USBD_SPEED_HS, &sample_hs_config);
    if (err) {
      LOG_ERR("Failed to add High-Speed configuration");
      return NULL;
    }

    err = usbd_register_all_classes(&sample_usbd, USBD_SPEED_HS, 1);
    if (err) {
      LOG_ERR("Failed to add register classes");
      return NULL;
    }

    sample_fix_code_triple(&sample_usbd, USBD_SPEED_HS);
  }

  /* doc configuration register start */
  err = usbd_add_configuration(&sample_usbd, USBD_SPEED_FS, &sample_fs_config);
  if (err) {
    LOG_ERR("Failed to add Full-Speed configuration");
    return NULL;
  }
  /* doc configuration register end */

  /* doc functions register start */
  err = usbd_register_all_classes(&sample_usbd, USBD_SPEED_FS, 1);
  if (err) {
    LOG_ERR("Failed to add register classes");
    return NULL;
  }
  /* doc functions register end */

  sample_fix_code_triple(&sample_usbd, USBD_SPEED_FS);

  if (msg_cb != NULL) {
    /* doc device init-and-msg start */
    err = usbd_msg_register_cb(&sample_usbd, msg_cb);
    if (err) {
      LOG_ERR("Failed to register message callback");
      return NULL;
    }
    /* doc device init-and-msg end */
  }

  if (IS_ENABLED(CONFIG_MYL_UTILS_USBD_20_EXTENSION_DESC)) {
    (void)usbd_device_set_bcd_usb(&sample_usbd, USBD_SPEED_FS, 0x0201);
    (void)usbd_device_set_bcd_usb(&sample_usbd, USBD_SPEED_HS, 0x0201);

    err = usbd_add_descriptor(&sample_usbd, &sample_usbext);
    if (err) {
      LOG_ERR("Failed to add USB 2.0 Extension Descriptor");
      return NULL;
    }
  }

  /* doc device init start */
  err = usbd_init(&sample_usbd);
  if (err) {
    LOG_ERR("Failed to initialize device support");
    return NULL;
  }
  /* doc device init end */

  return &sample_usbd;
}

#endif
