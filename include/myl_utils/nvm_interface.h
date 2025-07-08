#pragma once

#include "noncopyable.h"

class NvmInterface : NonCopyable<NvmInterface> {
 public:
  virtual int GetData(uint16_t key, uint8_t *data, size_t length) = 0;
  virtual int SetData(uint16_t key, const uint8_t *data, size_t length) = 0;
};