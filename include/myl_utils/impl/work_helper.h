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
#define PERIODIC_WORK(work, period, work_handler)                            \
  K_WORK_DEFINE(work, work_handler);                                         \
  void work##_timer_handler(struct k_timer *dummy) { k_work_submit(&work); } \
  K_TIMER_DEFINE(work##_timer, work##_timer_handler, NULL);                  \
  void work##_start() { k_timer_start(&work##_timer, period, period); }
