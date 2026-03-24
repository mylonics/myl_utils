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

// ---------------------------------------------------------------------------
// MYL_NOINLINE — prevent inlining of CRTP base-class dispatch methods
//                (SendPacket, ProcessCommand, GPIO Set/Toggle, etc.)
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
