#pragma once
#include <stddef.h>
#include <stdint.h>

// Returns the number of bytes written to buf, or 0 on error.
static inline size_t myl_get_device_unique_id(uint8_t *buf, size_t bufsize) {
#if defined(CONFIG_SOC_NRF52840) || defined(CONFIG_SOC_NRF5340_CPUAPP) || defined(CONFIG_SOC_NRF9160)
  // nRF: Use FICR->DEVICEADDR (MAC-like address)
  if (bufsize < 8) return 0;
  uint32_t addr0 = NRF_FICR->DEVICEADDR[0];
  uint32_t addr1 = NRF_FICR->DEVICEADDR[1];
  buf[0] = (addr0 >> 0) & 0xFF;
  buf[1] = (addr0 >> 8) & 0xFF;
  buf[2] = (addr0 >> 16) & 0xFF;
  buf[3] = (addr0 >> 24) & 0xFF;
  buf[4] = (addr1 >> 0) & 0xFF;
  buf[5] = (addr1 >> 8) & 0xFF;
  buf[6] = (addr1 >> 16) & 0xFF;
  buf[7] = (addr1 >> 24) & 0xFF;
  return 8;
#elif defined(CONFIG_SOC_SERIES_STM32)
// STM32: Use UID registers
#define STM32_UID_BASE 0x1FFF7590U
  if (bufsize < 12) return 0;
  volatile uint32_t *uid = (uint32_t *)STM32_UID_BASE;
  for (int i = 0; i < 3; ++i) {
    buf[i * 4 + 0] = (uid[i] >> 0) & 0xFF;
    buf[i * 4 + 1] = (uid[i] >> 8) & 0xFF;
    buf[i * 4 + 2] = (uid[i] >> 16) & 0xFF;
    buf[i * 4 + 3] = (uid[i] >> 24) & 0xFF;
  }
  return 12;
#elif defined(CONFIG_SOC_RP2040)
  // RP2040: Use flash unique ID (XIP)
  return 0;
#else
  // Unsupported platform
  return 0;
#endif
}

// Hashes the unique device ID buffer into a single int32 value
static inline int32_t myl_hash_device_unique_id_ll(void) {
  uint8_t idbuf[16] = {0};
  size_t idlen = myl_get_device_unique_id(idbuf, sizeof(idbuf));
  int32_t hash = 0;
  for (size_t i = 0; i < idlen; ++i) {
    hash = (hash * 31) ^ idbuf[i];
  }
  return hash;
}

static inline int32_t myl_hash_device_unique_id(void) {
  static const int32_t cached_hash = myl_hash_device_unique_id_ll();
  return cached_hash;
}
