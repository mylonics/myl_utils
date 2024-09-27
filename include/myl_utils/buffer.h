#pragma once

#include "cmsis_gcc.h"
#include "myl_utils/noncopyable.h"

template <class T, size_t size>
class CircularBuffer : NonCopyable<CircularBuffer<T, size>> {
 public:
  void Put(T item) {
    buf_[wloc_] = item;
    __DSB();
    wloc_ = (wloc_ + 1) % (size);
  }

  T Get() {
    T item = buf_[rloc_];
    __DSB();
    rloc_ = (rloc_ + 1) % (size);
    return item;
  }

  void Reset() {
    wloc_ = 0;
    rloc_ = 0;
  }

  bool Readable() { return wloc_ != rloc_; }

 private:
  T buf_[size];
  volatile size_t wloc_;
  volatile size_t rloc_;
};
