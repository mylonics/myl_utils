#pragma once

/** @brief Timed Work to call in interrupt context
 *
 *
 * @param name name of thread/work queue.
 * @param timer_handler function to call after period.
 * @param period timer period.
 *
 */
#define TIMED_WORK_WITH_HANDLER(name, timer_handler, period)                            \
  K_TIMER_DEFINE(name##_timer, timer_handler, NULL);                                    \
  void name##_start() { k_timer_start(&name##_timer, K_MSEC(period), K_MSEC(period)); } \
  void name##_stop() { k_timer_stop(&name##_timer); }

/** @brief Timed Work to call in interrupt context
 *
 *
 * @param name name of thread/work queue.
 * @param handler function to call after period.
 * @param period timer period.
 *
 */
#define TIMED_WORK(name, handler, period)                   \
  void name##_handler(struct k_timer *dummy) { handler(); } \
  TIMED_WORK_WITH_HANDLER(name, name##_handler, period)

/** @brief Created a work object, timer object, work function and timer function. Work will by called by the system work
 * queue once timer has expired. Timer needs to be started by calling work_start()
 *
 *
 * @param work name of work item.
 * @param period time in ms that work item should be called.
 * @param work_handler function that will be called periodically.
 *
 */
#define PERIODIC_WORK_WITH_HANDLER(work, period, work_handler)               \
  K_WORK_DEFINE(work, work_handler);                                         \
  void work##_timer_handler(struct k_timer *dummy) { k_work_submit(&work); } \
  TIMED_WORK_WITH_HANDLER(work, work##_timer_handler, period)

/** @brief Created a work object, timer object, work function and timer function. Work will by called by the system work
 * queue once timer has expired. Timer needs to be started by calling work_start()
 *
 *
 * @param work name of work item.
 * @param queue that work item should be called in.
 * @param period time in ms that work item should be called.
 * @param work_handler function that will be called periodically.
 *
 */
#define PERIODIC_WORK_WITH_HANDLER_TO_QUEUE(work, queue, period, work_handler)                \
  K_WORK_DEFINE(work, work_handler);                                                          \
  void work##_timer_handler(struct k_timer *dummy) { k_work_submit_to_queue(&queue, &work); } \
  TIMED_WORK_WITH_HANDLER(work, work##_timer_handler, period)

/** @brief Created a work object, timer object, work function and timer function. Work will by called by the system work
 * queue once timer has expired. Timer needs to be started by calling work_start()
 *
 *
 * @param work name of work item.
 * @param period time in ms that work item should be called.
 * @param handler function that will be called periodically.
 *
 */
#define PERIODIC_WORK(work, period, handler)                        \
  void work##_work_handler_func(struct k_work *work) { handler(); } \
  PERIODIC_WORK_WITH_HANDLER(work, period, work##_work_handler_func)

/** @brief Created a work object, timer object, work function and timer function. Work will by called by the system work
 * queue once timer has expired. Timer needs to be started by calling work_start()
 *
 *
 * @param work name of work item.
 * @param queue that work item should be called in.
 * @param period time in ms that work item should be called.
 * @param handler function that will be called periodically.
 *
 */
#define PERIODIC_WORK_TO_QUEUE(work, queue, period, handler)        \
  void work##_work_handler_func(struct k_work *work) { handler(); } \
  PERIODIC_WORK_WITH_HANDLER_TO_QUEUE(work, period, work##_work_handler_func)

/** @brief Created a delayed work object, timer object, work function and timer function. Work will by called by the
 * system work queue once timer has expired. Timer needs to be started by calling work_start()
 *
 *
 * @param work name of work item.
 * @param delay time that work item should be called in.
 * @param work_handler function that will be called after delay.
 *
 */
#define DELAYED_WORK_WITH_WORK_HANDLER(work, delay, work_handler) \
  K_WORK_DELAYABLE_DEFINE(work, work_handler);                    \
  void work##_start() { k_work_schedule(&work, K_MSEC(delay)); }

/** @brief Created a delayed work object, timer object, work function and timer function. Work will by called by the
 * system work queue once timer has expired. Timer needs to be started by calling work_start()
 *
 *
 * @param work name of work item.
 * @param delay time that work item should be called in.
 * @param handler function that will be called after delay.
 *
 */
#define DELAYED_WORK(work, delay, handler)                          \
  void work##_work_handler_func(struct k_work *work) { handler(); } \
  DELAYED_WORK_WITH_WORK_HANDLER(work, delay, work##_work_handler_func)

/** @brief Created a delayed work object, timer object, work function and timer function. Work will by called by the
 * system work queue once timer has expired. Timer needs to be started by calling work_start()
 *
 *
 * @param work name of work item.
 * @param queue that work item should be called in.
 * @param delay time that work item should be called in.
 * @param work_handler function that will be called after delay.
 *
 */
#define DELAYED_WORK_TO_QUEUE(work, queue, delay, work_handler) \
  K_WORK_DELAYABLE_DEFINE(work, work_handler);                  \
  void work##_start() { k_work_schedule_for_queue(&queue, &work, K_MSEC(delay)); }

#define PERIODIC_WORK_CLASS(object_name)                                 \
  void object_name##_func(struct k_work *work) { object_name.Runner(); } \
  PERIODIC_WORK(object_name##_wrk, K_MSEC(object_name.loop_time), object_name##_func)

/** @brief Created a thread and work queue
 *
 *
 * @param name name of thread/work queue.
 * @param stack_size size of thread/work queue.
 * @param priority thread priority.
 *
 */
#define CREATE_WORK_QUEUE(name, stack_size, priority)                                                    \
  K_THREAD_STACK_DEFINE(name##_area, stack_size);                                                        \
  struct k_work_q name##_work_q;                                                                         \
  void name##_work_q_start() {                                                                           \
    k_work_queue_init(&name##_work_q);                                                                   \
    k_work_queue_start(&name##_work_q, name##_area, K_THREAD_STACK_SIZEOF(name##_area), priority, NULL); \
  };
