#pragma once

#include "utils/noncopyable.h"

#define MODBUS_NVM_HELPER(nvm_interface, reg)  \
  nvm_interface.SetModbusRegisterFromNvm(reg); \
  reg.BindSetterFunction([this](uint16_t data) { return nvm_interface.UpdateNVMRegister(reg, data); });

class NvmInterface : NonCopyable<NvmInterface> {
 public:
  virtual bool GetData(uint16_t key, uint16_t &data) = 0;
  virtual bool GetData(uint32_t key, uint8_t *data, size_t length) = 0;

  virtual bool SetData(uint16_t key, uint16_t data) = 0;
  virtual bool SetData(uint32_t key, uint8_t *data, size_t length) = 0;

  template <class T>
  void SetModbusRegisterFromNvm(T &reg) {
    uint16_t data{};
    GetData(reg.GetNvmAddress(), data);
    reg.SetRegister(data);
  }

  template <class T>
  bool UpdateNVMRegister(T &reg, uint16_t data) {
    SetData(reg.GetNvmAddress(), data);
    uint16_t d;
    GetData(reg.GetNvmAddress(), d);
    return d == data;
  }
};