#pragma once

#include "buffer.h"

enum class UartBaud {
  B_1200 = 0,
  B_2400 = 1,
  B_4800 = 2,
  B_9600 = 3,
  B_14400 = 4,
  B_19200 = 5,
  B_28800 = 6,
  B_38400 = 7,
  B_57600 = 8,
  B_76800 = 9,
  B_115200 = 10,
  B_END
};

size_t UartBaudEnumToValue(UartBaud baud) {
  switch (baud) {
    case UartBaud::B_1200:
      return 1200;
      break;
    case UartBaud::B_2400:
      return 2400;
      break;
    case UartBaud::B_4800:
      return 4800;
      break;
    case UartBaud::B_14400:
      return 14400;
      break;
    case UartBaud::B_19200:
      return 19200;
      break;
    case UartBaud::B_28800:
      return 28800;
      break;
    case UartBaud::B_38400:
      return 38400;
      break;
    case UartBaud::B_57600:
      return 57600;
      break;
    case UartBaud::B_76800:
      return 76800;
      break;
    case UartBaud::B_115200:
      return 115200;
      break;
    case UartBaud::B_9600:
    default:
      return 9600;
      break;
  }
}

enum class UartStopBits { ONE = 0, TWO = 1 };

enum class UartParity { NONE = 0, EVEN = 1, ODD = 2 };

enum class UartDataBits {
  FIVE = 0,  /**< 5 data bits */
  SIX = 1,   /**< 6 data bits */
  SEVEN = 2, /**< 7 data bits */
  EIGHT = 3, /**< 8 data bits */
  NINE = 4,  /**< 9 data bits */
};

class SerialPort {
 public:
  virtual bool Start() = 0;

  virtual void PutC(char c) = 0;

  virtual void PutArray(uint8_t *c, size_t size) = 0;

  virtual bool Readable() = 0;

  virtual char GetC() = 0;

  virtual bool GetC(char &c, size_t timeout_ms) = 0;

  virtual size_t GetArray(uint8_t *c, size_t max_length) = 0;
};

class SerialRepeater {
 public:
  SerialRepeater(SerialPort &ser1, SerialPort &ser2) : ser1_{ser1}, ser2_{ser2} {}

  void Runner() {
    while (ser1_.Readable()) {
      ser2_.PutC(ser1_.GetC());
    }
    while (ser2_.Readable()) {
      ser1_.PutC(ser2_.GetC());
    }
  }

 private:
  SerialPort &ser1_;
  SerialPort &ser2_;
};