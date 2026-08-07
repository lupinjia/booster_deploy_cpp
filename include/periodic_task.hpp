/**
 * @file periodic_task.hpp
 * @author lupinjia (jason_1120202397@163.com)
 * @brief  Defines functions and classes for creating and managing periodic tasks in a real-time system.
           This implementation uses POSIX threads and real-time scheduling policies to ensure
           that tasks are executed at precise intervals with minimal jitter.
 * @version 0.1
 * @date 2026-01-05
 *
 * @copyright Copyright (c) 2026 Beijing Institute of Technology, Yasen Jia
 *
 * Ported from legged_sdk (include/common/periodic_task.hpp) into booster_deploy_cpp.
 * Changes vs. upstream:
 *   - added missing <string>, <memory>, <cstring> includes
 *   - removed unused <sys/mman.h>
 *   - fixed stale namespace-closing comment (was "agilepf")
 */
#pragma once
#ifndef PERIODIC_TASK_HPP
#define PERIODIC_TASK_HPP

#include <atomic>
#include <chrono>
#include <cstring>
#include <errno.h>
#include <functional>
#include <memory>
#include <pthread.h>
#include <string>
#include <thread>
#include <time.h>

#include "spdlog/spdlog.h"

namespace legged
{
namespace common
{
class PeriodicTask 
{
public:
  /**
   * @brief Construct a new Periodic Task object
   * 
   * @param task_name             Name of the task
   * @param interval_microseconds Interval between task executions in microseconds
   * @param task_function         The function to be executed periodically
   * @param cpu_id                CPU core to which the task thread should be pinned (-1 = no pinning)
   * @param policy                Scheduling policy (e.g., SCHED_OTHER, SCHED_FIFO, SCHED_RR)
   * @param priority              Scheduling priority
   */
  PeriodicTask(std::string task_name, float interval_microseconds, std::function<void()> task_function,
          int cpu_id = -1, int policy = SCHED_OTHER, int priority = 20)
    : period_ns_(static_cast<long long>(interval_microseconds * 1000.0f)), 
      task_name_(task_name), task_function_(task_function)
  {
    if(running_.load())
    {
      spdlog::warn("Task {} is already running.", task_name_);
      return;
    }
    thread_ = std::thread(&PeriodicTask::loopFunction, this);
    // set cpu affinity
    if(cpu_id != -1)
    {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(cpu_id, &cpuset);
      if (pthread_setaffinity_np(thread_.native_handle(), sizeof(cpu_set_t), &cpuset) != 0) {
        spdlog::error("Failed to set thread affinity for task {}", task_name_);
      }
    }
    // set scheduling policy and priority
    sched_param sch;
    sch.sched_priority = priority;
    if (pthread_setschedparam(thread_.native_handle(), policy, &sch) != 0)
    {
      spdlog::error("Failed to set thread priority for task {}", task_name_);
    }
    // set running flag
    running_.store(true);
  }

  /**
   * @brief Stop the periodic task
   * 
   */
  void stop()
  {
    if(!running_)
    {
      spdlog::warn("Tried to stop Task {}, but it wasn't running.", task_name_);
      return;
    }
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    spdlog::info("Task {} stopped.", task_name_);
  }

  /**
   * @brief Destroy the Periodic Task object
   * 
   */
  ~PeriodicTask()
  { 
    stop(); 
  }

private:
  /**
   * @brief The main loop function that runs the periodic task
   * 
   */
  void loopFunction()
  {
    // put _thread into sleep for a short time to ensure it starts after setting affinity and priority
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // print scheduling info once
    spdlog::info("Task {} run on cpu {}", task_name_, sched_getcpu());
    sched_param sch;
    int policy = 0;
    pthread_getschedparam(pthread_self(), &policy, &sch);
    spdlog::info("Task {} scheduling policy: {}, priority: {}", task_name_,
            (policy == SCHED_FIFO ? "SCHED_FIFO" : (policy == SCHED_RR ? "SCHED_RR" : "SCHED_OTHER")), sch.sched_priority);
          
    // get period start time and set period end time
    clock_gettime(CLOCK_MONOTONIC, &start_ts_);
    end_ts_ = start_ts_;
    end_ts_.tv_nsec += static_cast<long>(period_ns_ % 1000000000LL);
    end_ts_.tv_sec  += static_cast<time_t>(period_ns_ / 1000000000LL + end_ts_.tv_nsec / 1000000000L);
    end_ts_.tv_nsec %= 1000000000L;

    // main loop
    while (running_.load()) {
      clock_gettime(CLOCK_MONOTONIC, &start_ts_);
      task_function_(); // Call the task function

      // Sleep until next absolute time
      int err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &end_ts_, NULL);
      if (err != 0 && err != EINTR) {
        spdlog::error("clock_nanosleep failed for task {}: {}", task_name_, strerror(err));
      }

      // advance end time by period (absolute time)
      end_ts_.tv_nsec += static_cast<long>(period_ns_ % 1000000000LL);
      end_ts_.tv_sec  += static_cast<time_t>(period_ns_ / 1000000000LL + end_ts_.tv_nsec / 1000000000L);
      end_ts_.tv_nsec %= 1000000000L;
    }
  }

  long long period_ns_;                // period in ns
  // Use CLOCK_MONOTONIC and absolute sleeps via clock_nanosleep to reduce jitter.
  struct timespec start_ts_, end_ts_;
  std::atomic<bool> running_{false};    // if the task is running
  std::string task_name_;               // task name
  std::thread thread_;                  // task thread
  std::function<void()> task_function_; // task function
};

typedef std::shared_ptr<PeriodicTask> PeriodicTaskPtr;

/**
 * @brief Create a Periodic Task object
 * 
 * @param task_name              Name of the task
 * @param interval_microseconds  Interval between task executions in microseconds
 * @param task_function          Function to be executed periodically
 * @param cpuID                  CPU core ID to bind the task thread
 * @param policy                 Scheduling policy (e.g., SCHED_OTHER, SCHED_FIFO)
 * @param priority               Scheduling priority
 * @return PeriodicTaskPtr       Shared pointer to the created PeriodicTask object
 */
PeriodicTaskPtr createPeriodicTask(std::string task_name, float interval_microseconds, 
                                  std::function<void()> task_function,
                                  int cpuID = 0, int policy = SCHED_OTHER, int priority = 20)
{
  return std::make_shared<PeriodicTask>(task_name, interval_microseconds, task_function, cpuID, policy, priority);
}

} // namespace common

} // namespace legged

#endif // PERIODIC_TASK_HPP
