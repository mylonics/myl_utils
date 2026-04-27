#pragma once

/**
 * @file config.h
 * @brief Project-wide build configuration macros for myl_utils
 *
 * Define MYL_DEBUG in your build system (e.g. -DMYL_DEBUG) to enable
 * debug helpers. Define MYL_DEBUG_ISR to additionally keep ISR-related
 * buffer methods non-inlined for breakpoint debugging.
 *
 * Example CMake usage:
 *   target_compile_definitions(my_app PRIVATE MYL_DEBUG MYL_DEBUG_ISR)
 */

#include <cstddef>
#if defined(__cpp_lib_hardware_interference_size)
  #include <new>
#endif

// ---------------------------------------------------------------------------
// MYL_NOINLINE — prevent inlining of CRTP base-class dispatch methods
//                (SendSync, SendAsync, GPIO Set/Toggle, etc.)
//                so debuggers always have real symbols to break on.
// ---------------------------------------------------------------------------
#ifdef MYL_DEBUG
  #define MYL_NOINLINE __attribute__((noinline))
  #pragma message("MYL_NOINLINE enabled — CRTP methods will not be inlined (MYL_DEBUG)")
#else
  #define MYL_NOINLINE
#endif

// ---------------------------------------------------------------------------
// MYL_ISR_NOINLINE — same idea, but for ISR-shared ring buffer and DMA
//                    buffer methods (Put, Get, StartTransfer, etc.).
//                    Separated because these are hot paths and keeping them
//                    non-inlined has a measurable performance cost.
// ---------------------------------------------------------------------------
#ifdef MYL_DEBUG_ISR
  #define MYL_ISR_NOINLINE __attribute__((noinline))
  #pragma message("MYL_ISR_NOINLINE enabled — ISR buffer methods will not be inlined (MYL_DEBUG_ISR)")
#else
  #define MYL_ISR_NOINLINE
#endif

// ---------------------------------------------------------------------------
// kCacheLineSize — cache-line size used for alignment/padding.
//                  Override by defining MYL_CACHE_LINE_SIZE in your build.
//                  Falls back to std::hardware_destructive_interference_size
//                  when available, otherwise defaults to 64 bytes.
// ---------------------------------------------------------------------------
namespace myl_utils {
#if defined(MYL_CACHE_LINE_SIZE)
  inline constexpr size_t kCacheLineSize = MYL_CACHE_LINE_SIZE;
#elif defined(__cpp_lib_hardware_interference_size)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Winterference-size"
  inline constexpr size_t kCacheLineSize = std::hardware_destructive_interference_size;
  #pragma GCC diagnostic pop
#else
  inline constexpr size_t kCacheLineSize = 64u;
#endif
}  // namespace myl_utils
