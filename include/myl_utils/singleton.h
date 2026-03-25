#pragma once

namespace myl_utils {

template <typename T>
class Singleton {
 public:
  static T& getInstance() {
    static T instance;
    return instance;
  }

 protected:
  Singleton() {}
  ~Singleton() {}

 public:
  Singleton(Singleton const&) = delete;
  Singleton& operator=(Singleton const&) = delete;
  Singleton(Singleton&&) = delete;
  Singleton& operator=(Singleton&&) = delete;
};

}  // namespace myl_utils
