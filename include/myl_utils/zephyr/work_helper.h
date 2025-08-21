/**
 * @file work_helper.h
 * @brief Convenience macros for creating Zephyr timers, periodic and delayable work items.
 *
 * This header removes repetitive boilerplate around common scheduling patterns built on
 * Zephyr's k_timer, k_work, and k_work_delayable APIs.
 *
 * Macro catalog (choose by: periodic vs one-shot, handler form, need for custom queue, need to omit name):
 *
 * 1. Timers only (no work submission, runs in ISR context)
 *    - TIMED_WORK_WITH_HANDLER(NAME, TIMER_HANDLER, PERIOD_MS)
 *        Use when you already have a k_timer_expiry_t handler. Public: NAME_timer, NAME_start(), NAME_stop().
 *    - TIMED_WORK(NAME, HANDLER, PERIOD_MS)
 *        Use for a simple void(void) periodic callback. Wraps TIMED_WORK_WITH_HANDLER.
 *
 * 2. Periodic work (timer + k_work submitted each period)
 *    Core:
 *    - MYL_PERIODIC_WORK(WORK_NAME, PERIOD_MS, WORK_HANDLER, QUEUE_PTR)
 *        Lowest level. Provide a k_work_handler_t and an optional work queue pointer (NULL => system queue).
 *        Public: myl_WORK_NAME (k_work), WORK_NAME_timer, WORK_NAME_start(), WORK_NAME_stop().
 *    Wrappers (k_work_handler_t form):
 *    - PERIODIC_WORK_WITH_HANDLER(WORK_NAME, PERIOD_MS, WORK_HANDLER)
 *        System queue submission.
 *    - PERIODIC_WORK_WITH_HANDLER_TO_QUEUE(WORK_NAME, QUEUE, PERIOD_MS, WORK_HANDLER)
 *        Custom queue submission.
 *    Wrappers (void(void) handler form):
 *    - PERIODIC_WORK(WORK_NAME, PERIOD_MS, HANDLER)
 *        System queue; supply plain void(void) HANDLER.
 *    - PERIODIC_WORK_TO_QUEUE(WORK_NAME, QUEUE, PERIOD_MS, HANDLER)
 *        Custom queue; supply plain void(void) HANDLER.
 *    Name-eliding variants (use handler identifier as WORK_NAME; fastest to write, but only if unique):
 *    - PERIODIC_WORK_FOR(HANDLER, PERIOD_MS)
 *        System queue; HANDLER is void(void).
 *    - PERIODIC_WORK_FOR_TO_QUEUE(HANDLER, QUEUE, PERIOD_MS)
 *        Custom queue version of the above.
 *
 * 3. Delayable (single-shot) work
 *    Core:
 *    - MYL_DELAYED_WORK(WORK_NAME, WORK_HANDLER, QUEUE_PTR)
 *        Define a delayable work item; use myl_WORK_NAME_schedule(delay_ms) directly.
 *    Wrappers (k_work_handler_t form):
 *    - DELAYED_WORK_WITH_WORK_HANDLER(WORK_NAME, DELAY_MS, WORK_HANDLER)
 *        Adds myl_WORK_NAME_start(delay_ms) and myl_WORK_NAME_start_fixed(). Fixed delay captured by macro.
 *    - DELAYED_WORK_TO_QUEUE(WORK_NAME, QUEUE, DELAY_MS, WORK_HANDLER)
 *        Same but submits to custom queue.
 *    Wrappers (void(void) handler form):
 *    - DELAYED_WORK(WORK_NAME, DELAY_MS, HANDLER)
 *        System queue convenience creating internal k_work_handler_t wrapper.
 *
 * 4. C++ helper (legacy / adapter)
 *    - PERIODIC_WORK_CLASS(OBJECT_NAME)
 *        For objects exposing Runner() + loop_time field; generates a periodic work using those.
 *
 * 5. Infrastructure helpers
 *    - CREATE_WORK_QUEUE(NAME, STACK_SIZE, PRIORITY)
 *        Define and start a dedicated work queue (call myl_NAME_work_q_start()).
 *    - CREATE_THREAD_FUNCTION(NAME, STACK_SIZE, PRIORITY, FUNCTION)
 *        Define a thread that runs FUNCTION() once on start.
 *
 * Selection guidance:
 *  - Need only a timer callback: use TIMED_WORK*.
 *  - Need work context each period: pick a PERIODIC_* variant.
 *      * Have a k_work_handler_t already? Use PERIODIC_WORK_WITH_HANDLER*.
 *      * Only a simple void(void) function? Use PERIODIC_WORK*.
 *      * Want to skip naming? Use PERIODIC_WORK_FOR* (ensure handler name uniqueness).
 *  - Need one-shot deferrable execution: choose a DELAYED_* variant.
 *      * Use MYL_DELAYED_WORK directly for maximum control; wrappers add start helpers + fixed delay.
 *  - In C++ with objects exposing Runner()/loop_time: PERIODIC_WORK_CLASS.
 *
 * Symbol naming summary (timers use NAME_timer; work structs use myl_<name>):
 *   TIMED_WORK_WITH_HANDLER(NAME, ...) => NAME_timer, NAME_start(), NAME_stop()
 *   MYL_PERIODIC_WORK(WORK_NAME, ...) => myl_WORK_NAME, WORK_NAME_timer, WORK_NAME_start(), WORK_NAME_stop()
 *   MYL_DELAYED_WORK(WORK_NAME, ...)  => myl_WORK_NAME, myl_WORK_NAME_schedule()
 *   Wrappers add myl_WORK_NAME_start(), myl_WORK_NAME_start_fixed() where relevant.
 *
 * Best practices:
 *  - Handlers must be short / non-blocking (no long loops, sleeps, heavy allocations).
 *  - Period change requires stop() then restart (period is compile-time here).
 *  - Consider adding explicit cancel / is_pending helpers if your usage pattern needs runtime introspection.
 *
 * Timing units: All *_MS parameters are integer milliseconds; K_MSEC performed inside macros.
 * (Future enhancement: accept k_timeout_t directly.)
 *
 * Thread-safety: start/stop/schedule are safe per Zephyr API; ensure lifetime of generated objects.
 *
 * Caveats:
 *  - Fixed periods/delays baked into macro instantiation (except manual schedule(delay_ms)).
 *  - Timers from periodic macros currently not prefixed (legacy). Name collisions are unlikely but possible.
 */

#pragma once

/**
 * @brief Create a periodic Zephyr timer.
 *
 * Public symbols generated:
 *   - k_timer NAME_timer
 *   - void NAME_start(void)
 *   - void NAME_stop(void)
 *
 * @param NAME          Base name.
 * @param TIMER_HANDLER Handler of type k_timer_expiry_t (void (*)(struct k_timer *)).
 * @param PERIOD_MS     Period in ms (applied via K_MSEC inside).
 */
/* ------------------------------------------------------------------------------------------------
 * Timed work (timer only)
 * ------------------------------------------------------------------------------------------------ */
#define TIMED_WORK_WITH_HANDLER(NAME, TIMER_HANDLER, PERIOD_MS)                                                       \
  K_TIMER_DEFINE(myl_##NAME##_timer, TIMER_HANDLER, NULL);                                                            \
  static inline void NAME##_start(void) { k_timer_start(&myl_##NAME##_timer, K_MSEC(PERIOD_MS), K_MSEC(PERIOD_MS)); } \
  static inline void NAME##_stop(void) { k_timer_stop(&myl_##NAME##_timer); }

/**
 * @brief Convenience wrapper around TIMED_WORK_WITH_HANDLER for a void(void) callback.
 * Public symbols (same as TIMED_WORK_WITH_HANDLER): NAME_timer, NAME_start(), NAME_stop().
 * Internal: static myl_NAME_handler(struct k_timer *).
 *
 * @param NAME       Base name.
 * @param HANDLER    void(void) function executed each period.
 * @param PERIOD_MS  Period in ms.
 */
#define TIMED_WORK(NAME, HANDLER, PERIOD_MS)            \
  static void myl_##NAME##_handler(struct k_timer *t) { \
    (void)t;                                            \
    HANDLER();                                          \
  }                                                     \
  TIMED_WORK_WITH_HANDLER(NAME, myl_##NAME##_handler, PERIOD_MS)

/**
 * @brief Periodic work (system work queue) scheduled via timer.
 *
 * Generates:
 *   - k_work:    myl_@p WORK_NAME
 *   - k_timer:   myl_@p WORK_NAME _timer
 *   - helpers:   myl_@p WORK_NAME _start(), myl_@p WORK_NAME _stop()
 *
 * @param WORK_NAME    Base name.
 * @param PERIOD_MS    Interval in ms.
 * @param WORK_HANDLER k_work_handler_t.
 */
/* ------------------------------------------------------------------------------------------------
 * Unified periodic work macro (queue optional)
 * ------------------------------------------------------------------------------------------------ */
/**
 * @brief Core periodic work macro.
 *
 * Public symbols generated:
 *   - struct k_work myl_WORK_NAME
 *   - k_timer WORK_NAME_timer
 *   - void WORK_NAME_start(void)
 *   - void WORK_NAME_stop(void)
 * Internal: static myl_WORK_NAME_timer_handler(struct k_timer *)
 *
 * @param WORK_NAME     Base name.
 * @param PERIOD_MS     Interval in ms.
 * @param WORK_HANDLER  k_work_handler_t.
 * @param QUEUE_PTR     Pointer to struct k_work_q or NULL for system queue.
 */
#define MYL_PERIODIC_WORK(WORK_NAME, PERIOD_MS, WORK_HANDLER, QUEUE_PTR) \
  K_WORK_DEFINE(myl_##WORK_NAME, WORK_HANDLER);                          \
  static void myl_##WORK_NAME##_timer_handler(struct k_timer *t) {       \
    (void)t;                                                             \
    if ((QUEUE_PTR) != NULL) {                                           \
      (void)k_work_submit_to_queue((QUEUE_PTR), &myl_##WORK_NAME);       \
    } else {                                                             \
      (void)k_work_submit(&myl_##WORK_NAME);                             \
    }                                                                    \
  }                                                                      \
  TIMED_WORK_WITH_HANDLER(WORK_NAME, myl_##WORK_NAME##_timer_handler, PERIOD_MS)

/**
 * @brief Periodic work submitted to a specified work queue.
 *
 * @param WORK_NAME    Base name.
 * @param QUEUE        Destination struct k_work_q identifier.
 * @param PERIOD_MS    Interval in ms.
 * @param WORK_HANDLER k_work_handler_t.
 */
/* Backward compatible wrappers (produce same public symbol set as MYL_PERIODIC_WORK) */
#define PERIODIC_WORK_WITH_HANDLER(WORK_NAME, PERIOD_MS, WORK_HANDLER) \
  MYL_PERIODIC_WORK(WORK_NAME, PERIOD_MS, WORK_HANDLER, NULL)

#define PERIODIC_WORK_WITH_HANDLER_TO_QUEUE(WORK_NAME, QUEUE, PERIOD_MS, WORK_HANDLER) \
  MYL_PERIODIC_WORK(WORK_NAME, PERIOD_MS, WORK_HANDLER, &(QUEUE))

/**
 * @brief Periodic work (system queue) for a void(void) handler.
 * Public symbols: myl_WORK_NAME, WORK_NAME_timer, WORK_NAME_start(), WORK_NAME_stop().
 *
 * @param WORK_NAME Base name.
 * @param PERIOD_MS Interval in ms.
 * @param HANDLER   void(void) callback.
 */
#define PERIODIC_WORK(WORK_NAME, PERIOD_MS, HANDLER)             \
  static void myl_##WORK_NAME##_work_wrapper(struct k_work *w) { \
    (void)w;                                                     \
    HANDLER();                                                   \
  }                                                              \
  MYL_PERIODIC_WORK(WORK_NAME, PERIOD_MS, myl_##WORK_NAME##_work_wrapper, NULL)

/**
 * @brief Periodic work (custom queue) for a void(void) handler.
 * Public symbols: myl_WORK_NAME, WORK_NAME_timer, WORK_NAME_start(), WORK_NAME_stop().
 *
 * @param WORK_NAME Base name.
 * @param QUEUE     Destination queue.
 * @param PERIOD_MS Interval in ms.
 * @param HANDLER   void(void) callback.
 */
#define PERIODIC_WORK_TO_QUEUE(WORK_NAME, QUEUE, PERIOD_MS, HANDLER) \
  static void myl_##WORK_NAME##_work_wrapper(struct k_work *w) {     \
    (void)w;                                                         \
    HANDLER();                                                       \
  }                                                                  \
  MYL_PERIODIC_WORK(WORK_NAME, PERIOD_MS, myl_##WORK_NAME##_work_wrapper, &(QUEUE))

/**
 * @brief Periodic work (system queue) using only an existing handler name.
 * The handler's identifier is reused as the WORK_NAME base so you don't need to
 * supply a separate name. Only safe to use if the handler identifier is a
 * simple function name (not a pointer variable) and you will not create a
 * different periodic work instance with the same handler.
 * Public symbols: myl_HANDLER, HANDLER_timer, HANDLER_start(), HANDLER_stop().
 *
 * @param HANDLER    void(void) function to run each period.
 * @param PERIOD_MS  Interval in ms.
 */
#define PERIODIC_WORK_FOR(HANDLER, PERIOD_MS)                  \
  static void myl_##HANDLER##_work_wrapper(struct k_work *w) { \
    (void)w;                                                   \
    HANDLER();                                                 \
  }                                                            \
  MYL_PERIODIC_WORK(HANDLER, PERIOD_MS, myl_##HANDLER##_work_wrapper, NULL)

/**
 * @brief Periodic work (custom queue) using only an existing handler name.
 * Same as PERIODIC_WORK_FOR but submits to the specified work queue instead of
 * the system queue. The handler identifier becomes the WORK_NAME base.
 * Public symbols: myl_HANDLER, HANDLER_timer, HANDLER_start(), HANDLER_stop().
 *
 * @param HANDLER    void(void) function to run each period.
 * @param QUEUE      Destination queue (struct k_work_q identifier, not pointer).
 * @param PERIOD_MS  Interval in ms.
 */
#define PERIODIC_WORK_FOR_TO_QUEUE(HANDLER, QUEUE, PERIOD_MS)  \
  static void myl_##HANDLER##_work_wrapper(struct k_work *w) { \
    (void)w;                                                   \
    HANDLER();                                                 \
  }                                                            \
  MYL_PERIODIC_WORK(HANDLER, PERIOD_MS, myl_##HANDLER##_work_wrapper, &(QUEUE))

/**
 * @brief Delayable work (single-shot) with explicit k_work_handler_t.
 *
 * Generates:
 *   - k_work_delayable: myl_@p WORK_NAME
 *   - start helper:     myl_@p WORK_NAME _start()
 *
 * @param WORK_NAME    Base name.
 * @param DELAY_MS     Delay in ms.
 * @param WORK_HANDLER k_work_handler_t.
 */
/* ------------------------------------------------------------------------------------------------
 * Unified delayable work macro (queue optional)
 * ------------------------------------------------------------------------------------------------ */
/**
 * @brief Core delayable work macro.
 *
 * Public symbols generated:
 *   - struct k_work_delayable myl_WORK_NAME
 *   - int myl_WORK_NAME_schedule(uint32_t delay_ms)
 *
 * @param WORK_NAME     Base name.
 * @param WORK_HANDLER  k_work_handler_t.
 * @param QUEUE_PTR     Pointer to struct k_work_q or NULL for system queue.
 */
#define MYL_DELAYED_WORK(WORK_NAME, WORK_HANDLER, QUEUE_PTR)                             \
  K_WORK_DELAYABLE_DEFINE(myl_##WORK_NAME, WORK_HANDLER);                                \
  static inline int myl_##WORK_NAME##_schedule(uint32_t delay_ms) {                      \
    if ((QUEUE_PTR) != NULL) {                                                           \
      return k_work_schedule_for_queue((QUEUE_PTR), &myl_##WORK_NAME, K_MSEC(delay_ms)); \
    }                                                                                    \
    return k_work_schedule(&myl_##WORK_NAME, K_MSEC(delay_ms));                          \
  }

/**
 * @brief Delayable work with explicit handler and fixed DELAY_MS convenience.
 * Public symbols: myl_WORK_NAME, myl_WORK_NAME_schedule(), myl_WORK_NAME_start(), myl_WORK_NAME_start_fixed().
 *
 * @param WORK_NAME    Base name.
 * @param DELAY_MS     Delay in ms.
 * @param WORK_HANDLER k_work_handler_t.
 */
#define DELAYED_WORK_WITH_WORK_HANDLER(WORK_NAME, DELAY_MS, WORK_HANDLER)                                       \
  MYL_DELAYED_WORK(WORK_NAME, WORK_HANDLER, NULL);                                                              \
  static inline int myl_##WORK_NAME##_start(uint32_t delay_ms) { return myl_##WORK_NAME##_schedule(delay_ms); } \
  static inline int myl_##WORK_NAME##_start_fixed(void) { return myl_##WORK_NAME##_schedule(DELAY_MS); }

#define DELAYED_WORK(WORK_NAME, DELAY_MS, HANDLER)          \
  static void myl_##WORK_NAME##_handler(struct k_work *w) { \
    (void)w;                                                \
    HANDLER();                                              \
  }                                                         \
  DELAYED_WORK_WITH_WORK_HANDLER(WORK_NAME, DELAY_MS, myl_##WORK_NAME##_handler)

/**
 * @brief Delayable work (void handler) wrapper.
 * Public symbols: myl_WORK_NAME, myl_WORK_NAME_schedule(), myl_WORK_NAME_start(), myl_WORK_NAME_start_fixed().
 *
 * @param WORK_NAME Base name.
 * @param DELAY_MS  Delay in ms.
 * @param HANDLER   void(void) callback.
 */
#define DELAYED_WORK_TO_QUEUE(WORK_NAME, QUEUE, DELAY_MS, WORK_HANDLER)                                         \
  MYL_DELAYED_WORK(WORK_NAME, WORK_HANDLER, &(QUEUE));                                                          \
  static inline int myl_##WORK_NAME##_start(uint32_t delay_ms) { return myl_##WORK_NAME##_schedule(delay_ms); } \
  static inline int myl_##WORK_NAME##_start_fixed(void) { return myl_##WORK_NAME##_schedule(DELAY_MS); }

/**
 * @brief C++ adapter for objects exposing Runner() & loop_time.
 * Public symbols: myl_OBJECT_NAME_func (static), myl_OBJECT_NAME_wrk, OBJECT_NAME_wrk_timer, OBJECT_NAME_wrk_start(),
 * OBJECT_NAME_wrk_stop().
 */
#define PERIODIC_WORK_CLASS(OBJECT_NAME)                   \
  static void myl_##OBJECT_NAME##_func(struct k_work *w) { \
    (void)w;                                               \
    OBJECT_NAME.Runner();                                  \
  }                                                        \
  PERIODIC_WORK(OBJECT_NAME##_wrk, K_MSEC(OBJECT_NAME.loop_time), myl_##OBJECT_NAME##_func)

/**
 * @brief Delayable work submitted to a specific queue (explicit handler).
 * Public symbols: myl_WORK_NAME, myl_WORK_NAME_schedule(), myl_WORK_NAME_start(), myl_WORK_NAME_start_fixed().
 * (Already documented above; duplicate brief removed.)
 */

/**
 * @brief Create a dedicated work queue and start helper.
 * Public symbols: myl_NAME_area (stack), myl_NAME_work_q, myl_NAME_work_q_start().
 *
 * @param NAME       Base name.
 * @param STACK_SIZE Stack size in bytes.
 * @param PRIORITY   Thread priority.
 */
#define CREATE_WORK_QUEUE(NAME, STACK_SIZE, PRIORITY)                                                               \
  K_THREAD_STACK_DEFINE(myl_##NAME##_area, STACK_SIZE);                                                             \
  struct k_work_q myl_##NAME##_work_q;                                                                              \
  static inline void myl_##NAME##_work_q_start(void) {                                                              \
    k_work_queue_init(&myl_##NAME##_work_q);                                                                        \
    k_work_queue_start(&myl_##NAME##_work_q, myl_##NAME##_area, K_THREAD_STACK_SIZEOF(myl_##NAME##_area), PRIORITY, \
                       NULL);                                                                                       \
  }

/**
 * @brief Create a Zephyr thread executing a simple void(void) FUNCTION once at start.
 * Public symbols: myl_NAME_tid, myl_NAME_tfunction (static entry).
 *
 * @param NAME       Base name.
 * @param STACK_SIZE Stack size in bytes.
 * @param PRIORITY   Thread priority.
 * @param FUNCTION   void(void) function body.
 */
#define CREATE_THREAD_FUNCTION(NAME, STACK_SIZE, PRIORITY, FUNCTION) \
  static void myl_##NAME##_tfunction(void *a, void *b, void *c) {    \
    (void)a;                                                         \
    (void)b;                                                         \
    (void)c;                                                         \
    FUNCTION();                                                      \
  }                                                                  \
  K_THREAD_DEFINE(myl_##NAME##_tid, STACK_SIZE, myl_##NAME##_tfunction, NULL, NULL, NULL, PRIORITY, 0, 0);
