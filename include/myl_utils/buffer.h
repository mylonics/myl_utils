#pragma once

#include <cstddef>
#include <cstring>
#include <type_traits>
#include <utility>

#include "config.h"
#include "myl_utils/noncopyable.h"

#ifdef MYL_DEBUG
  #include <cassert>
#endif

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
// store_release / store_relaxed / store_seq_cst.
// ---------------------------------------------------------------------------

#if defined(MYL_BUFFER_ATOMIC_STD)

class AtomicIndex {
 public:
  constexpr AtomicIndex() noexcept : v_{0} {}
  size_t load_acquire() const noexcept { return v_.load(std::memory_order_acquire); }
  size_t load_relaxed() const noexcept { return v_.load(std::memory_order_relaxed); }
  void   store_release(size_t x) noexcept { v_.store(x, std::memory_order_release); }
  void   store_relaxed(size_t x) noexcept { v_.store(x, std::memory_order_relaxed); }
  void   store_seq_cst(size_t x) noexcept { v_.store(x, std::memory_order_seq_cst); }
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
  void   store_seq_cst(size_t x) noexcept { v_.store(x, etl::memory_order_seq_cst); }
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
  void   store_seq_cst(size_t x) noexcept { __atomic_store_n(&v_, x, __ATOMIC_SEQ_CST); }
  static constexpr bool is_lock_free() noexcept {
    return __atomic_always_lock_free(sizeof(size_t), 0);
  }
 private:
  mutable size_t v_;  // mutable: required so const load methods can pass &v_ to __atomic_load_n
};

#endif

}  // namespace detail

/**
 * @brief Non-owning producer-side view of a CircularBuffer.
 *
 * Exposes only the write interface: Put, TryPut, Writable, FreeSpace.
 * Obtained via CircularBuffer::MakeProducer(). Remains valid while the
 * originating CircularBuffer instance is alive.
 *
 * Must be used by at most ONE producer at a time (SPSC contract).
 */
template <typename T>
class BufferProducer {
 public:
  BufferProducer(T* buf, detail::AtomicIndex* wloc,
                 detail::AtomicIndex* rloc, size_t mask) noexcept
      : buf_(buf), wloc_(wloc), rloc_(rloc), mask_(mask) {}

  MYL_ISR_NOINLINE void Put(const T& item) noexcept {
    const size_t w    = wloc_->load_relaxed();
    const size_t next = (w + 1) & mask_;
    if (next == rloc_->load_acquire()) {
      // Buffer full — silently drop the oldest item by advancing the read
      // index. Safe on single-core (ISR producer + thread consumer): if the
      // consumer already loaded the old r, it will store the same value back.
      // NOT safe for true SMP with concurrent producer and consumer cores.
      rloc_->store_relaxed((next + 1) & mask_);
    }
    buf_[w] = item;
    wloc_->store_release(next);
  }

  MYL_ISR_NOINLINE void Put(T&& item) noexcept {
    const size_t w    = wloc_->load_relaxed();
    const size_t next = (w + 1) & mask_;
    if (next == rloc_->load_acquire()) {
      // Buffer full — silently drop the oldest item by advancing the read
      // index. Safe on single-core (ISR producer + thread consumer): if the
      // consumer already loaded the old r, it will store the same value back.
      // NOT safe for true SMP with concurrent producer and consumer cores.
      rloc_->store_relaxed((next + 1) & mask_);
    }
    buf_[w] = std::move(item);
    wloc_->store_release(next);
  }

  [[nodiscard]] MYL_ISR_NOINLINE bool TryPut(const T& item) noexcept {
    const size_t w = wloc_->load_relaxed();
    const size_t next = (w + 1) & mask_;
    if (next == rloc_->load_acquire()) return false;
    buf_[w] = item;
    wloc_->store_release(next);
    return true;
  }

  [[nodiscard]] MYL_ISR_NOINLINE bool TryPut(T&& item) noexcept {
    const size_t w = wloc_->load_relaxed();
    const size_t next = (w + 1) & mask_;
    if (next == rloc_->load_acquire()) return false;
    buf_[w] = std::move(item);
    wloc_->store_release(next);
    return true;
  }

  bool Writable() const noexcept {
    const size_t w = wloc_->load_relaxed();
    return ((w + 1) & mask_) != rloc_->load_acquire();
  }

  /// Snapshot of available free slots. May increase as the consumer removes items.
  size_t FreeSpace() const noexcept {
    const size_t w = wloc_->load_relaxed();
    const size_t r = rloc_->load_acquire();
    return mask_ - ((w - r) & mask_);
  }

  /// Write up to count items from data in a single atomic transaction
  /// (one acquire-load and one release-store, regardless of count).
  /// Returns the number of items written; may be less than count if the
  /// buffer lacks space. Never overwrites — use Put() for single-item
  /// overwrite-on-full behaviour.
  size_t TryPutN(const T* data, size_t count) noexcept {
    const size_t w    = wloc_->load_relaxed();
    const size_t r    = rloc_->load_acquire();
    const size_t free = mask_ - ((w - r) & mask_);
    if (count > free) count = free;
    if (count == 0) return 0;
    const size_t cap  = mask_ + 1;  // N
    const size_t seg1 = cap - w;    // elements from w to end of array
    if (count <= seg1) {
      for (size_t i = 0; i < count; ++i) buf_[w + i] = data[i];
    } else {
      for (size_t i = 0; i < seg1; ++i) buf_[w + i] = data[i];
      const size_t seg2 = count - seg1;
      for (size_t i = 0; i < seg2; ++i) buf_[i] = data[seg1 + i];
    }
    wloc_->store_release((w + count) & mask_);
    return count;
  }

 private:
  T*                   buf_;
  detail::AtomicIndex* wloc_;
  detail::AtomicIndex* rloc_;
  size_t               mask_;
};

/**
 * @brief Non-owning consumer-side view of a CircularBuffer.
 *
 * Exposes only the read interface: Get, TryGet, Readable, Size.
 * Obtained via CircularBuffer::MakeConsumer(). Remains valid while the
 * originating CircularBuffer instance is alive.
 *
 * Must be used by at most ONE consumer at a time (SPSC contract).
 */
template <typename T>
class BufferConsumer {
 public:
  BufferConsumer(T* buf, detail::AtomicIndex* wloc,
                 detail::AtomicIndex* rloc, size_t mask) noexcept
      : buf_(buf), wloc_(wloc), rloc_(rloc), mask_(mask) {}

  MYL_ISR_NOINLINE T Get() noexcept {
#ifdef MYL_DEBUG
    assert(Readable());
#endif
    const size_t r = rloc_->load_relaxed();
    T item = buf_[r];
    rloc_->store_release((r + 1) & mask_);
    return item;
  }

  [[nodiscard]] MYL_ISR_NOINLINE bool TryGet(T& item) noexcept {
    const size_t w = wloc_->load_acquire();
    const size_t r = rloc_->load_relaxed();
    if (w == r) return false;
    item = buf_[r];
    rloc_->store_release((r + 1) & mask_);
    return true;
  }

  bool Readable() const noexcept {
    return wloc_->load_acquire() != rloc_->load_relaxed();
  }

  /// Snapshot of currently-buffered item count.
  /// May increase concurrently; will not decrease while called from the consumer.
  size_t Size() const noexcept {
    const size_t w = wloc_->load_acquire();
    const size_t r = rloc_->load_relaxed();
    return (w - r) & mask_;
  }

  /// Read and remove up to count items into out in a single atomic transaction
  /// (one acquire-load and one release-store, regardless of count).
  /// Returns the number of items read; may be less than count if fewer
  /// items are currently available.
  size_t GetN(T* out, size_t count) noexcept {
    const size_t w     = wloc_->load_acquire();
    const size_t r     = rloc_->load_relaxed();
    const size_t avail = (w - r) & mask_;
    if (count > avail) count = avail;
    if (count == 0) return 0;
    const size_t cap  = mask_ + 1;  // N
    const size_t seg1 = cap - r;    // elements from r to end of array
    if (count <= seg1) {
      for (size_t i = 0; i < count; ++i) out[i] = buf_[r + i];
    } else {
      for (size_t i = 0; i < seg1; ++i) out[i] = buf_[r + i];
      const size_t seg2 = count - seg1;
      for (size_t i = 0; i < seg2; ++i) out[seg1 + i] = buf_[i];
    }
    rloc_->store_release((r + count) & mask_);
    return count;
  }

 private:
  T*                   buf_;
  detail::AtomicIndex* wloc_;
  detail::AtomicIndex* rloc_;
  size_t               mask_;
};

/**
 * @brief Lock-free single-producer / single-consumer power-of-2 ring buffer.
 *
 * Owns the storage (buf_, wloc_, rloc_). All read/write logic lives in the
 * view types returned by MakeProducer() / MakeConsumer(). CircularBuffer
 * itself only manages lifetime and provides Reset().
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
 * Overflow behaviour:
 *   When Put() is called on a full buffer it silently drops the oldest item
 *   (advances the read index by one) before writing the new item. The buffer
 *   therefore always contains the most recent N-1 items. Use TryPut() if you
 *   need to detect and reject writes on full.
 *
 * Concurrency contract:
 *   - At most ONE producer may use the BufferProducer view at a time.
 *   - At most ONE consumer may use the BufferConsumer view at a time.
 *   - Producer and consumer may be in different threads / ISR contexts.
 *   - Put() overwrite-on-full is safe on single-core (ISR + thread) only.
 *     True SMP with both cores active simultaneously is not supported for
 *     the overwrite path.
 */
template <class T, size_t N>
class CircularBuffer : NonCopyable<CircularBuffer<T, N>> {
  static_assert((N & (N - 1)) == 0, "CircularBuffer size must be a power of 2");
  static_assert(N >= 2, "CircularBuffer size must be at least 2");
  // BufferProducer/BufferConsumer do copy/move assignment and copy construction.
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
  /// Obtain a producer-side view. Valid while this CircularBuffer is alive.
  /// Pass by value — trivially copyable (4 pointer-sized fields).
  BufferProducer<T> MakeProducer() noexcept {
    return {buf_, &wloc_, &rloc_, mask_};
  }

  /// Obtain a consumer-side view. Same lifetime rules as MakeProducer().
  BufferConsumer<T> MakeConsumer() noexcept {
    return {buf_, &wloc_, &rloc_, mask_};
  }

  /// Reset both indices to zero.
  /// PRECONDITION: no Put()/TryPut() or Get()/TryGet() call may be in progress
  /// on any core. External synchronisation (e.g. thread join, RTOS task suspend)
  /// must be applied before this call and before either side resumes. The seq_cst
  /// stores ensure both zeroed writes are coherently ordered and visible to all
  /// cores without requiring a separate fence from the caller.
  void Reset() noexcept {
    wloc_.store_seq_cst(0);
    rloc_.store_seq_cst(0);
  }

  static constexpr size_t Capacity() noexcept { return N - 1; }

 private:
  T buf_[N]{};
  // Placed on separate cache lines to prevent false sharing between the
  // producer (writes wloc_) and consumer (writes rloc_) on SMP targets.
  alignas(kCacheLineSize) detail::AtomicIndex wloc_;
  alignas(kCacheLineSize) detail::AtomicIndex rloc_;
};

// ---------------------------------------------------------------------------

/**
 * @brief Single-context power-of-2 ring buffer — no atomics, no ISR sharing.
 *
 * All operations must be called from the same execution context (same thread,
 * same RTOS task, same cooperative scheduler slot). Do NOT share an instance
 * between an ISR and a task, or between two tasks without external
 * synchronisation — use CircularBuffer with BufferProducer/BufferConsumer
 * for cross-context producer/consumer patterns.
 *
 * Because there is no concurrent access, no atomic operations or memory
 * barriers are emitted and no cache-line padding is needed. This makes
 * LocalBuffer suited for targets without atomic support (AVR, bare RISC-V
 * without the A extension, soft-core FPGAs) and for cooperative schedulers
 * where the code-size and RAM overhead of AtomicIndex is unwanted.
 *
 * Overflow behaviour of Put():
 *   When the buffer is full, Put() silently drops the oldest item before
 *   writing the new one. Use TryPut() / TryPutN() to detect and reject
 *   writes when full.
 *
 * Zero-copy read:
 *   ReadableSpan() returns a pointer and length for the first contiguous
 *   readable region. Call Consume(n) to advance the read index without
 *   copying. If the readable data wraps the ring, a second call to
 *   ReadableSpan() after consuming the first segment returns the remainder.
 */
template <typename T, size_t N>
class LocalBuffer : NonCopyable<LocalBuffer<T, N>> {
  static_assert((N & (N - 1)) == 0, "LocalBuffer size must be a power of 2");
  static_assert(N >= 2,             "LocalBuffer size must be at least 2");
  static_assert(std::is_trivially_copyable<T>::value,
                "LocalBuffer<T>: T must be trivially copyable (enables memcpy bulk paths "
                "and ensures safe use in same-context embedded code)");
  static constexpr size_t kMask = N - 1;

 public:
  /// Pointer + length describing one contiguous region inside the ring.
  struct Span {
    const T* ptr;
    size_t   len;
  };

  // -------------------------------------------------------------------------
  // Single-item interface
  // -------------------------------------------------------------------------

  /// Write item; overwrites oldest entry if full.
  void Put(const T& item) noexcept {
    const size_t next = (w_ + 1) & kMask;
    if (next == r_) r_ = (next + 1) & kMask;  // full: drop oldest
    buf_[w_] = item;
    w_ = next;
  }

  /// Write item (move); overwrites oldest entry if full.
  void Put(T&& item) noexcept {
    const size_t next = (w_ + 1) & kMask;
    if (next == r_) r_ = (next + 1) & kMask;
    buf_[w_] = std::move(item);
    w_ = next;
  }

  /// Write item only if space is available. Returns true on success.
  [[nodiscard]] bool TryPut(const T& item) noexcept {
    const size_t next = (w_ + 1) & kMask;
    if (next == r_) return false;
    buf_[w_] = item;
    w_ = next;
    return true;
  }

  /// Write item (move) only if space is available. Returns true on success.
  [[nodiscard]] bool TryPut(T&& item) noexcept {
    const size_t next = (w_ + 1) & kMask;
    if (next == r_) return false;
    buf_[w_] = std::move(item);
    w_ = next;
    return true;
  }

  /// Read and remove the oldest item. Behaviour is undefined if empty.
  T Get() noexcept {
#ifdef MYL_DEBUG
    assert(Readable());
#endif
    T item = buf_[r_];
    r_ = (r_ + 1) & kMask;
    return item;
  }

  /// Read and remove the oldest item into out. Returns false if empty.
  [[nodiscard]] bool TryGet(T& out) noexcept {
    if (w_ == r_) return false;
    out = buf_[r_];
    r_ = (r_ + 1) & kMask;
    return true;
  }

  // -------------------------------------------------------------------------
  // Bulk transfer interface
  // -------------------------------------------------------------------------

  /// Write up to count items from data. Returns items actually written (0 if
  /// full). Never overwrites — use Put() in a loop for that.
  size_t TryPutN(const T* data, size_t count) noexcept {
    const size_t free = kMask - ((w_ - r_) & kMask);
    if (count > free) count = free;
    if (count == 0) return 0;
    const size_t seg1 = N - w_;  // elements from w_ to end of array
    if (count <= seg1) {
      std::memcpy(buf_ + w_, data, count * sizeof(T));
    } else {
      std::memcpy(buf_ + w_, data,          seg1 * sizeof(T));
      std::memcpy(buf_,      data + seg1, (count - seg1) * sizeof(T));
    }
    w_ = (w_ + count) & kMask;
    return count;
  }

  /// Read and remove up to count items into out. Returns items actually read.
  size_t GetN(T* out, size_t count) noexcept {
    const size_t avail = (w_ - r_) & kMask;
    if (count > avail) count = avail;
    if (count == 0) return 0;
    const size_t seg1 = N - r_;  // elements from r_ to end of array
    if (count <= seg1) {
      std::memcpy(out, buf_ + r_, count * sizeof(T));
    } else {
      std::memcpy(out,        buf_ + r_, seg1 * sizeof(T));
      std::memcpy(out + seg1, buf_,      (count - seg1) * sizeof(T));
    }
    r_ = (r_ + count) & kMask;
    return count;
  }

  // -------------------------------------------------------------------------
  // Zero-copy read interface
  // -------------------------------------------------------------------------

  /// Return the first contiguous readable region without consuming it.
  /// If all buffered data is contiguous, len == Size(). If the data wraps,
  /// len covers only the pre-wrap segment; call again after Consume(len)
  /// to obtain the second (post-wrap) segment.
  /// Returns {nullptr, 0} if empty.
  Span ReadableSpan() const noexcept {
    const size_t avail = (w_ - r_) & kMask;
    if (avail == 0) return {nullptr, 0};
    const size_t seg1 = N - r_;
    return {buf_ + r_, avail < seg1 ? avail : seg1};
  }

  /// Advance the read index by n without copying any data.
  /// PRECONDITION: n <= Size(). Violating this corrupts the buffer state.
  void Consume(size_t n) noexcept {
#ifdef MYL_DEBUG
    assert(n <= ((w_ - r_) & kMask));
#endif
    r_ = (r_ + n) & kMask;
  }

  // -------------------------------------------------------------------------
  // State queries
  // -------------------------------------------------------------------------

  bool   Readable()  const noexcept { return w_ != r_; }
  bool   Writable()  const noexcept { return ((w_ + 1) & kMask) != r_; }
  size_t Size()      const noexcept { return (w_ - r_) & kMask; }
  size_t FreeSpace() const noexcept { return kMask - ((w_ - r_) & kMask); }
  static constexpr size_t Capacity() noexcept { return N - 1; }

  void Reset() noexcept { w_ = 0; r_ = 0; }

 private:
  T      buf_[N]{};
  size_t w_{0};
  size_t r_{0};
};

}  // namespace myl_utils
