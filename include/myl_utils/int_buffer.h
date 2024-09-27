#pragma once

#include <string.h>

#include <functional>

#include "buffer.h"
#include "utils/noncopyable.h"

template <class T, size_t size>
    class InterruptTxBuffer : NonCopyable < InterruptTxBuffer<T, size> {
 public:
  InterruptTxBuffer(std::function<void(T *, size_t)> transmit_function) : transmit_function_(transmit_function) {}

  void Put(T item) {
    loading_ = true;
    __DSB();
    buf_[wloc_] = item;
    __DSB();
    wloc_ = (wloc_ + 1) % (size);
    loading_ = false;
    if (!transmitting_) {
      StartTransfer();
    }
  }

  bool Full() { return wloc_ + size; }

  void Reset() { wloc_ = 0; }

  void StartTransfer() {
    if (!loading_ && wloc_) {
      memcpy(transmitbuf_, buf_, wloc_);
      transmit_function_(transmitbuf_, wloc_);
      transmitting_ = true;
      //__DSB();
      wloc_ = 0;
    } else {
      transmitting_ = false;
    }
  }

 private:
  T buf_[size];
  T transmitbuf_[size];

  std::function<void(T *, size_t)> transmit_function_;

  volatile size_t wloc_{};
  bool transmitting_{};
  bool loading_{};
};

template <class T, size_t size>
class InterruptRxCircularBuffer {
 public:
  InterruptRxCircularBuffer(std::function<void(T *)> enable_rx_interrupt) : enable_rx_interrupt_(enable_rx_interrupt) {}

  void Start() { Prime(); }

  T Get() { return buffer.Get(); }

  void Reset() { buffer.Reset(); }

  bool Readable() { return buffer.Readable(); }

  void Put() {
    buffer.Put(incoming_byte_);
    Prime();
  }

 private:
  T incoming_byte_{};
  CircularBuffer<T, size> buffer;

  void Prime() { enable_rx_interrupt_(&incoming_byte_); }

  std::function<void(T *)> enable_rx_interrupt_;
};

template <class T, size_t size>
class DmaRxCircularBuffer {
 public:
  DmaRxCircularBuffer(std::function<void(T *)> start_dma_receive, std::function<size_t()> get_write_loc)
      : start_dma_receive_(start_dma_receive), get_write_loc_(get_write_loc) {}

  void Start() { start_dma_receive_(buf_); }

  T Get() {
    T item = buf_[rloc_];
    __DSB();
    rloc_ = (rloc_ + 1) % (size);
    return item;
  }

  bool Readable() {
    UpdateWLoc();
    return wloc_ != rloc_;
  }

 private:
  void UpdateWLoc() { wloc_ = get_write_loc_(); }

  T incoming_byte_{};
  T buf_[size];

  size_t rloc_;
  size_t wloc_;

  std::function<void(T *b)> start_dma_receive_;
  std::function<size_t()> get_write_loc_;
};
