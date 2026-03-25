#pragma once

#include "noncopyable.h"

namespace myl_utils {

template <typename T>
class Singleton : NonCopyable<Singleton<T>> {
 public:
  static T& getInstance() {
    static T instance;
    return instance;
  }

 protected:
  Singleton() {}
  ~Singleton() {}
};

}  // namespace myl_utils
