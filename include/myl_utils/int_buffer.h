#pragma once

#include <string.h>

#include "buffer.h"
#include "myl_utils/noncopyable.h"

namespace myl_utils {

template <class T, size_t size>
class InterruptTxBuffer : NonCopyable<InterruptTxBuffer<T, size>> {
  static_assert((size & (size - 1)) == 0, "InterruptTxBuffer size must be a power of 2");
  static constexpr size_t mask_ = size - 1;

 public:
  explicit InterruptTxBuffer(void (*transmit_function)(T *, size_t))
      : transmit_function_(transmit_function) {}

  void Put(T item) {
    loading_ = true;
    __DSB();
    buf_[wloc_] = item;
    __DSB();
    wloc_ = (wloc_ + 1) & mask_;
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

  void (*transmit_function_)(T *, size_t);

  volatile size_t wloc_{};
  bool transmitting_{};
  bool loading_{};
};

template <class T, size_t size>
class InterruptRxCircularBuffer {
 public:
  explicit InterruptRxCircularBuffer(void (*enable_rx_interrupt)(T *))
      : enable_rx_interrupt_(enable_rx_interrupt) {}

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

  void (*enable_rx_interrupt_)(T *);
};

template <class T, size_t size>
class DmaRxCircularBuffer {
  static_assert((size & (size - 1)) == 0, "DmaRxCircularBuffer size must be a power of 2");
  static constexpr size_t mask_ = size - 1;

 public:
  DmaRxCircularBuffer(void (*start_dma_receive)(T *), size_t (*get_write_loc)())
      : start_dma_receive_(start_dma_receive), get_write_loc_(get_write_loc) {}

  void Start() { start_dma_receive_(buf_); }

  T Get() {
    T item = buf_[rloc_];
    __DSB();
    rloc_ = (rloc_ + 1) & mask_;
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

  void (*start_dma_receive_)(T *);
  size_t (*get_write_loc_)();
};

}  // namespace myl_utils
