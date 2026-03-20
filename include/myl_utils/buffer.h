#pragma once

// #include "cmsis_gcc.h"
#include "myl_utils/noncopyable.h"

namespace myl_utils {

template <class T, size_t size>
class CircularBuffer : NonCopyable<CircularBuffer<T, size>> {
  static_assert((size & (size - 1)) == 0, "CircularBuffer size must be a power of 2");
  static constexpr size_t mask_ = size - 1;

 public:
  void Put(T item) {
    buf_[wloc_] = item;
    //__DSB();
    wloc_ = (wloc_ + 1) & mask_;
  }

  T Get() {
    T item = buf_[rloc_];
    //__DSB();
    rloc_ = (rloc_ + 1) & mask_;
    return item;
  }

  void Reset() {
    wloc_ = 0;
    rloc_ = 0;
  }

  bool Readable() { return wloc_ != rloc_; }

  bool Writable() { return ((wloc_ + 1) & mask_) != rloc_; }

 private:
  T buf_[size];
  volatile size_t wloc_;
  volatile size_t rloc_;
};

}  // namespace myl_utils
