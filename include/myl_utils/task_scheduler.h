#pragma once

#include "buffer.h"
#include "utils/noncopyable.h"

class Task : NonCopyable<Task> {
 public:
  uint64_t period_ms_;
  std::function<void()> runner_;

  uint64_t last_run_{};

  Task(uint64_t period_ms, std::function<void()> runner) : period_ms_(period_ms), runner_(runner) {}

  void RunPeriodic(uint64_t current_ms) {
    if (current_ms > (last_run_ + period_ms_)) {
      last_run_ = current_ms;
      runner_();
    }
  }

  void UpdatePeriod(uint64_t period_ms) { period_ms_ = period_ms; }

  void RunNow() const { runner_(); }
};

template <size_t size>
class TaskHandler : NonCopyable<TaskHandler<size>> {
 private:
  std::function<uint64_t()> get_ms_;
  Task *tasks[size];
  size_t number_of_tasks_{};

 public:
  TaskHandler(std::function<uint64_t()> get_ms) : get_ms_(get_ms) {}

  bool AddTask(Task &t) {
    if (number_of_tasks_ < size) {
      tasks[number_of_tasks_] = &t;
      number_of_tasks_++;
      return true;
    }
    return false;
  }

  void Run() const {
    for (size_t i = 0; i < number_of_tasks_; i++) {
      tasks[i]->RunPeriodic(get_ms_());
    }
  }
};
