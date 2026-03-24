#pragma once

// #include "cmsis_gcc.h"
#include "config.h"
#include "myl_utils/noncopyable.h"

namespace myl_utils {

template <class T, size_t size>
class CircularBuffer : NonCopyable<CircularBuffer<T, size>> {
  static_assert((size & (size - 1)) == 0, "CircularBuffer size must be a power of 2");
  static constexpr size_t mask_ = size - 1;

 public:
  MYL_ISR_NOINLINE void Put(T item) {
    buf_[wloc_] = item;
    //__DSB();
    wloc_ = (wloc_ + 1) & mask_;
  }

  MYL_ISR_NOINLINE T Get() {
    T item = buf_[rloc_];
    //__DSB();
    rloc_ = (rloc_ + 1) & mask_;
    return item;
  }

  MYL_ISR_NOINLINE void Reset() {
    wloc_ = 0;
    rloc_ = 0;
  }

  MYL_ISR_NOINLINE bool Readable() { return wloc_ != rloc_; }

  MYL_ISR_NOINLINE bool Writable() { return ((wloc_ + 1) & mask_) != rloc_; }

 private:
  T buf_[size];
  volatile size_t wloc_;
  volatile size_t rloc_;
};

}  // namespace myl_utils
