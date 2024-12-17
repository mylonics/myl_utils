#pragma once

/** @brief Created a work object, timer object, work function and timer function. Timer needs to be started by calling
 * work_start()
 *
 *
 * @param work name of work item.
 * @param period time in ms that work item should be called.
 * @param work_handler function that will be called periodically.
 *
 * @return as with k_work_submit_to_queue().
 */
#define PERIODIC_WORK(work, period, work_handler)                                       \
  K_WORK_DEFINE(work, work_handler);                                                    \
  void work##_timer_handler(struct k_timer *dummy) { k_work_submit(&work); }            \
  K_TIMER_DEFINE(work##_timer, work##_timer_handler, NULL);                             \
  void work##_start() { k_timer_start(&work##_timer, K_MSEC(period), K_MSEC(period)); } \
  void work##_stop() { k_timer_stop(&work##_timer); }

#define PERIODIC_WORK_TO_QUEUE(work, queue, period, work_handler)                             \
  K_WORK_DEFINE(work, work_handler);                                                          \
  void work##_timer_handler(struct k_timer *dummy) { k_work_submit_to_queue(&queue, &work); } \
  K_TIMER_DEFINE(work##_timer, work##_timer_handler, NULL);                                   \
  void work##_start() { k_timer_start(&work##_timer, K_MSEC(period), K_MSEC(period)); }

#define DELAYED_WORK(work, delay, work_handler) \
  K_WORK_DELAYABLE_DEFINE(work, work_handler);  \
  void work##_start() { k_work_schedule(&work, K_MSEC(delay)); }

#define PERIODIC_WORK_CLASS(object_name)                                 \
  void object_name##_func(struct k_work *work) { object_name.Runner(); } \
  PERIODIC_WORK(object_name##_wrk, K_MSEC(object_name.loop_time),        \
                object_name##_func)

#define CREATE_WORK_QUEUE(name, stack_size, priority)                                                    \
  K_THREAD_STACK_DEFINE(name##_area, stack_size);                                                        \
  struct k_work_q name##_work_q;                                                                         \
  void name##_work_q_start()                                                                             \
  {                                                                                                      \
    k_work_queue_init(&name##_work_q);                                                                   \
    k_work_queue_start(&name##_work_q, name##_area, K_THREAD_STACK_SIZEOF(name##_area), priority, NULL); \
  };

#define TIMED_WORK(name, handler, period)                   \
  void name##_handler(struct k_timer *dummy) { handler(); } \
  K_TIMER_DEFINE(name##_timer, name##_handler, NULL);       \
  void name##_timer_start() { k_timer_start(&name##_timer, K_MSEC(period), K_MSEC(period)); }
