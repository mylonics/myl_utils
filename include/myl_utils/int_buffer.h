#pragma once

#include <string.h>

#include "buffer.h"
#include "config.h"
#include "myl_utils/noncopyable.h"

namespace myl_utils {

template <class T, size_t size>
class InterruptTxBuffer : NonCopyable<InterruptTxBuffer<T, size>> {
  static_assert((size & (size - 1)) == 0, "InterruptTxBuffer size must be a power of 2");
  static constexpr size_t mask_ = size - 1;

 public:
  explicit InterruptTxBuffer(void (*transmit_function)(T *, size_t))
      : transmit_function_(transmit_function) {}

  MYL_ISR_NOINLINE void Put(T item) {
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

  bool Full() { return ((wloc_ + 1) & mask_) == 0; }

  void Reset() { wloc_ = 0; }

  MYL_ISR_NOINLINE void StartTransfer() {
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
      : producer_(buffer_), consumer_(buffer_), enable_rx_interrupt_(enable_rx_interrupt) {}

  MYL_ISR_NOINLINE void Start() { Prime(); }

  // Call from task/consumer context only.
  MYL_ISR_NOINLINE T Get() { return consumer_.Get(); }

  MYL_ISR_NOINLINE void Reset() { buffer_.Reset(); }

  // Call from task/consumer context only.
  MYL_ISR_NOINLINE bool Readable() { return consumer_.Readable(); }

  // Call from ISR/producer context only.
  MYL_ISR_NOINLINE void Put() {
    producer_.Put(incoming_byte_);
    Prime();
  }

 private:
  T incoming_byte_{};
  CircularBuffer<T, size> buffer_;
  CircularBufferProducer<T, size> producer_;
  CircularBufferConsumer<T, size> consumer_;

  void Prime() { enable_rx_interrupt_(&incoming_byte_); }

  void (*enable_rx_interrupt_)(T *);
};

template <class T, size_t size>
class DmaRxCircularBuffer : NonCopyable<DmaRxCircularBuffer<T, size>> {
  static_assert((size & (size - 1)) == 0, "DmaRxCircularBuffer size must be a power of 2");
  static constexpr size_t mask_ = size - 1;

 public:
  DmaRxCircularBuffer(void (*start_dma_receive)(T *), size_t (*get_write_loc)())
      : start_dma_receive_(start_dma_receive), get_write_loc_(get_write_loc) {}

  MYL_ISR_NOINLINE void Start() { start_dma_receive_(buf_); }

  MYL_ISR_NOINLINE T Get() {
    T item = buf_[rloc_];
    __DSB();
    rloc_ = (rloc_ + 1) & mask_;
    return item;
  }

  MYL_ISR_NOINLINE bool Readable() {
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
