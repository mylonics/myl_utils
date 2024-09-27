#pragma once

#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>

#include "myl_utils/nvm_interface.h"

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

/** @brief This class sets up a nonvolatile memory. It expects a storage partition to be defined in the board dts file.
 *
 */
class NvmController : public NvmInterface {
 public:
  explicit NvmController() : NvmInterface{} {
    int rc = 0;
    struct flash_pages_info info;

    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
      printk("Flash device %s is not ready\n", fs.flash_device->name);
    }

    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
      printk("Flash device %s is not ready\n", fs.flash_device->name);
    }
    fs.offset = NVS_PARTITION_OFFSET;
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
      printk("Unable to get page info\n");
    }
    fs.sector_size = info.size;
    fs.sector_count = 3U;

    rc = nvs_mount(&fs);
    if (rc) {
      printk("Flash Init failed %d \n", rc);
    }
  }

  bool GetData(uint16_t key, uint16_t &data) {
    int rc = nvs_read(&fs, key, &data, sizeof(data));
    return rc == sizeof(data);
  }

  bool SetData(uint16_t key, uint16_t data) {
    int rc = nvs_write(&fs, key, &data, sizeof(data));
    return rc >= 0;
  }

  bool GetData(uint32_t address, uint8_t *data, size_t length) {
    int rc = nvs_read(&fs, address, &data, length);
    return rc == (int)length;
  }

  bool SetData(uint32_t address, uint8_t *data, size_t length) {
    int rc = nvs_write(&fs, address, &data, length);
    return rc >= 0;
  }

 private:
  struct nvs_fs fs;
};