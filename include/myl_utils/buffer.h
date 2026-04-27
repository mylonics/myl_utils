#pragma once

#include <cstddef>
#include <new>
#include <type_traits>
#include <utility>

#include "config.h"
#include "myl_utils/noncopyable.h"

// ---------------------------------------------------------------------------
// Atomic backend selection
// ---------------------------------------------------------------------------
// Preference order:
//   1. std::atomic                  (full hosted C++ library available)
//   2. etl::atomic                  (Embedded Template Library — for freestanding
//                                    toolchains where <atomic> is missing)
//   3. GCC/Clang __atomic_ builtins (last resort — works on any target with a
//                                    GCC-compatible compiler, no library needed)
//
// To force a backend, predefine one of:
//   MYL_BUFFER_ATOMIC_STD, MYL_BUFFER_ATOMIC_ETL, MYL_BUFFER_ATOMIC_BUILTIN
// ---------------------------------------------------------------------------

#if !defined(MYL_BUFFER_ATOMIC_STD) && !defined(MYL_BUFFER_ATOMIC_ETL) && \
    !defined(MYL_BUFFER_ATOMIC_BUILTIN)
  #if __has_include(<atomic>)
    #define MYL_BUFFER_ATOMIC_STD 1
  #elif __has_include(<etl/atomic.h>)
    #define MYL_BUFFER_ATOMIC_ETL 1
  #elif defined(__GNUC__) || defined(__clang__)
    #define MYL_BUFFER_ATOMIC_BUILTIN 1
  #else
    #error "myl_utils CircularBuffer: no atomic backend available — define MYL_BUFFER_ATOMIC_BUILTIN if your compiler supports __atomic_ builtins"
  #endif
#endif

#if defined(MYL_BUFFER_ATOMIC_STD)
  #include <atomic>
#elif defined(MYL_BUFFER_ATOMIC_ETL)
  #include <etl/atomic.h>
#endif

namespace myl_utils {
namespace detail {

// ---------------------------------------------------------------------------
// AtomicIndex — minimal acquire/release atomic for the SPSC ring buffer.
// Uniform interface regardless of backend: load_acquire / load_relaxed /
// store_release / store_relaxed.
// ---------------------------------------------------------------------------

#if defined(MYL_BUFFER_ATOMIC_STD)

class AtomicIndex {
 public:
  constexpr AtomicIndex() noexcept : v_{0} {}
  size_t load_acquire() const noexcept { return v_.load(std::memory_order_acquire); }
  size_t load_relaxed() const noexcept { return v_.load(std::memory_order_relaxed); }
  void   store_release(size_t x) noexcept { v_.store(x, std::memory_order_release); }
  void   store_relaxed(size_t x) noexcept { v_.store(x, std::memory_order_relaxed); }
  static constexpr bool is_lock_free() noexcept {
    return std::atomic<size_t>::is_always_lock_free;
  }
 private:
  std::atomic<size_t> v_;
};

#elif defined(MYL_BUFFER_ATOMIC_ETL)

class AtomicIndex {
 public:
  constexpr AtomicIndex() noexcept : v_{0} {}
  size_t load_acquire() const noexcept { return v_.load(etl::memory_order_acquire); }
  size_t load_relaxed() const noexcept { return v_.load(etl::memory_order_relaxed); }
  void   store_release(size_t x) noexcept { v_.store(x, etl::memory_order_release); }
  void   store_relaxed(size_t x) noexcept { v_.store(x, etl::memory_order_relaxed); }
  // etl::atomic<size_t> uses __atomic_* builtins for integral types, but those
  // builtins may fall back to libatomic (mutex-based) on targets without native
  // atomic instructions (e.g. AVR, bare RISC-V without A extension). Query the
  // compiler directly — same check the builtin backend uses.
  static constexpr bool is_lock_free() noexcept {
    return __atomic_always_lock_free(sizeof(size_t), 0);
  }
 private:
  etl::atomic<size_t> v_;
};

#else  // MYL_BUFFER_ATOMIC_BUILTIN

class AtomicIndex {
 public:
  constexpr AtomicIndex() noexcept : v_{0} {}
  size_t load_acquire() const noexcept { return __atomic_load_n(&v_, __ATOMIC_ACQUIRE); }
  size_t load_relaxed() const noexcept { return __atomic_load_n(&v_, __ATOMIC_RELAXED); }
  void   store_release(size_t x) noexcept { __atomic_store_n(&v_, x, __ATOMIC_RELEASE); }
  void   store_relaxed(size_t x) noexcept { __atomic_store_n(&v_, x, __ATOMIC_RELAXED); }
  static constexpr bool is_lock_free() noexcept {
    return __atomic_always_lock_free(sizeof(size_t), 0);
  }
 private:
  mutable size_t v_;  // mutable: required so const load methods can pass &v_ to __atomic_load_n
};

#endif

// Cache-line size for false-sharing prevention between producer/consumer indices.
// Uses std::hardware_destructive_interference_size when available (C++17),
// otherwise defaults to 64 bytes, which is safe for Cortex-A, x86, and RISC-V.
// inline constexpr avoids ODR violations when included from multiple TUs.
#if defined(__cpp_lib_hardware_interference_size)
  inline constexpr size_t kCacheLineSize = std::hardware_destructive_interference_size;
#else
  inline constexpr size_t kCacheLineSize = 64u;
#endif

}  // namespace detail

/**
 * @brief Lock-free single-producer / single-consumer power-of-2 ring buffer.
 *
 * Memory ordering uses acquire/release semantics through a thin atomic
 * abstraction (std::atomic, etl::atomic, or __atomic_ builtins — selected
 * automatically by header availability). The compiler emits the correct
 * barrier (or none) for each target ISA:
 *   - ARMv7-A / ARMv8 / Cortex-M7+M33 multi-core  -> `dmb ish`
 *   - Cortex-M0/M3/M4 single-core                 -> no barrier emitted
 *   - RISC-V (RV32/RV64 with A extension)         -> `fence rw,rw`
 *   - x86 (TSO)                                   -> no barrier emitted
 *
 * Concurrency contract:
 *   - At most ONE producer may call Put() / Writable().
 *   - At most ONE consumer may call Get() / Readable().
 *   - Producer and consumer may be in different threads / ISR contexts.
 *
 * Caller must check Writable() before Put() and Readable() before Get();
 * Put() does not bounds-check and Get() returns indeterminate data when empty.
 */
template <class T, size_t N>
class CircularBuffer : NonCopyable<CircularBuffer<T, N>> {
  static_assert((N & (N - 1)) == 0, "CircularBuffer size must be a power of 2");
  static_assert(N >= 2, "CircularBuffer size must be at least 2");
  // All Put/Get paths do copy/move assignment; all Get() paths do copy construction.
  // Every operation must be nothrow because Put/Get may run inside an ISR.
  static_assert(std::is_nothrow_copy_constructible<T>::value &&
                std::is_nothrow_copy_assignable<T>::value &&
                std::is_nothrow_move_assignable<T>::value,
                "CircularBuffer<T>: T must be nothrow copy-constructible, copy-assignable, "
                "and move-assignable (required for ISR-safe Put/Get)");
  static_assert(detail::AtomicIndex::is_lock_free(),
                "CircularBuffer requires lock-free atomic size_t on this target");
  static constexpr size_t mask_ = N - 1;

 public:
  /// Write one item (copy). Caller must verify Writable() first.
  MYL_ISR_NOINLINE void Put(const T& item) noexcept {
    const size_t w = wloc_.load_relaxed();
    buf_[w] = item;
    // Release ensures the data store above is visible to any consumer that
    // observes the new wloc_ via an acquire load.
    wloc_.store_release((w + 1) & mask_);
  }

  /// Write one item (move). Caller must verify Writable() first.
  MYL_ISR_NOINLINE void Put(T&& item) noexcept {
    const size_t w = wloc_.load_relaxed();
    buf_[w] = std::move(item);
    wloc_.store_release((w + 1) & mask_);
  }

  /// Write one item (copy) if space is available. Returns true on success.
  /// Preferred over Writable()+Put() — single coherent check-and-write.
  MYL_ISR_NOINLINE bool TryPut(const T& item) noexcept {
    const size_t w = wloc_.load_relaxed();
    const size_t next = (w + 1) & mask_;
    if (next == rloc_.load_acquire()) return false;
    buf_[w] = item;
    wloc_.store_release(next);
    return true;
  }

  /// Write one item (move) if space is available. Returns true on success.
  /// On false return the buffer is full and 'item' has NOT been moved from.
  MYL_ISR_NOINLINE bool TryPut(T&& item) noexcept {
    const size_t w = wloc_.load_relaxed();
    const size_t next = (w + 1) & mask_;
    if (next == rloc_.load_acquire()) return false;
    buf_[w] = std::move(item);
    wloc_.store_release(next);
    return true;
  }

  /// Read and remove one item. Caller must verify Readable() first.
  MYL_ISR_NOINLINE T Get() noexcept {
    const size_t r = rloc_.load_relaxed();
    T item = buf_[r];
    // Release ensures the data read above happens-before any producer that
    // observes the new rloc_ via an acquire load (e.g. for full-detection).
    rloc_.store_release((r + 1) & mask_);
    return item;
  }

  /// Read and remove one item if available. Returns true on success.
  /// Preferred over Readable()+Get() — single coherent check-and-read.
  MYL_ISR_NOINLINE bool TryGet(T& item) noexcept {
    const size_t w = wloc_.load_acquire();
    const size_t r = rloc_.load_relaxed();
    if (w == r) return false;
    item = buf_[r];
    rloc_.store_release((r + 1) & mask_);
    return true;
  }

  /// Reset both indices to zero.
  /// PRECONDITION: the caller must guarantee that no Put()/TryPut() or
  /// Get()/TryGet() call is in progress on any core.  The caller is also
  /// responsible for issuing its own synchronisation (e.g. thread join,
  /// RTOS task suspend, or an explicit seq_cst fence) before and after
  /// calling Reset() so that both zeroed stores are visible to all cores
  /// before either the producer or consumer resumes.
  void Reset() noexcept {
    wloc_.store_release(0);
    rloc_.store_release(0);
  }

  /// Consumer-side check: are there items available to Get()?
  bool Readable() const noexcept {
    // Acquire on wloc_ pairs with the release store in Put().
    return wloc_.load_acquire() != rloc_.load_relaxed();
  }

  /// Producer-side check: is there room for one more Put()?
  bool Writable() const noexcept {
    const size_t w = wloc_.load_relaxed();
    // Acquire on rloc_ pairs with the release store in Get().
    return ((w + 1) & mask_) != rloc_.load_acquire();
  }

  /// Snapshot of currently-buffered count (may increase concurrently as the
  /// producer adds items; will not decrease while called from the consumer).
  /// Must only be called from the CONSUMER side: load_acquire on wloc_ pairs
  /// with the release store in Put(). Calling from the producer side breaks
  /// the acquire/release pairing on weakly-ordered architectures (ARM, RISC-V).
  size_t Size() const noexcept {
    const size_t w = wloc_.load_acquire();
    const size_t r = rloc_.load_relaxed();
    return (w - r) & mask_;
  }

  /// Snapshot of available free slots (may increase as the consumer removes items).
  /// Must only be called from the PRODUCER side: load_acquire on rloc_ pairs
  /// with the release store in Get(). Calling from the consumer side breaks
  /// the acquire/release pairing on weakly-ordered architectures (ARM, RISC-V).
  size_t FreeSpace() const noexcept {
    const size_t w = wloc_.load_relaxed();
    // Acquire pairs with the release store in Get().
    const size_t r = rloc_.load_acquire();
    return mask_ - ((w - r) & mask_);
  }

  static constexpr size_t Capacity() noexcept { return N - 1; }

 private:
  T buf_[N]{};
  // Placed on separate cache lines to prevent false sharing between the
  // producer (writes wloc_) and consumer (writes rloc_) on SMP targets.
  alignas(detail::kCacheLineSize) detail::AtomicIndex wloc_;
  alignas(detail::kCacheLineSize) detail::AtomicIndex rloc_;
};

}  // namespace myl_utils
