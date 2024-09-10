// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONCEALER__AUTOWARE_STATE_DISPATCHER_HPP_
#define CONCEALER__AUTOWARE_STATE_DISPATCHER_HPP_

#include <functional>
#include <future>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>

template <typename TAutowareState>
class AutowareStateDispatcher
{
public:
  AutowareStateDispatcher() = default;
  virtual ~AutowareStateDispatcher() = default;

  void onAutowareState(const TAutowareState & msg, const rclcpp::Time & now)
  {
    if (auto pre_message = std::exchange(last_message, msg);
        getState(pre_message) != getState(msg)) {
      onStateChanged(getState(pre_message), getState(msg));
    }
    executeTask(getState(msg), now);
  }

  std::string getState(const TAutowareState & message) const { return message.state; }

  void registerTask(
    const std::string state, std::function<void()> task,
    rclcpp::Duration interval =
      rclcpp::Duration::from_nanoseconds(std::numeric_limits<rcl_duration_value_t>::max()))
  {
    tasks[state].push_back(Task(std::move(task), interval));
  }

  void unregisterTask(std::string state) { tasks.erase(state); }

  void requestUnregisterAllTasks() { task_clear_requested.store(true); }

private:
  void unregisterAllTasks() { tasks.clear(); }

  void onStateChanged(const std::string & pre_state, const std::string & state)
  {
    std::cout << "AutowareState changed from " << pre_state << " to " << state << std::endl;
  }

  void executeTask(std::string state, const rclcpp::Time & now)
  {
    if (task_clear_requested.load()) {
      unregisterAllTasks();
      task_clear_requested.store(false);
    }
    if (auto it = tasks.find(state); it != tasks.end()) {
      for (auto & task : it->second) {
        task(now);
      }
    }
  }

  TAutowareState last_message;

  struct Task
  {
    std::function<void()> task_function;

    rclcpp::Duration interval;

    rclcpp::Time next_execution;

    std::future<void> future;

    Task(std::function<void()> t, rclcpp::Duration i)
    : task_function(std::move(t)), interval(i), next_execution(rclcpp::Time(0, 0, RCL_ROS_TIME))
    {
    }

    void operator()(const rclcpp::Time & now)
    {
      if (
        future.valid() and future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        future.get();
      }

      if (now >= next_execution) {
        if (not future.valid()) {
          // pop new task
          auto task = std::packaged_task<void()>(task_function);
          future = task.get_future();
          auto worker = std::thread(std::move(task));
          worker.detach();
        }

        int64_t duration_nanoseconds = now.nanoseconds() - next_execution.nanoseconds();
        int64_t interval_nanoseconds = interval.nanoseconds();
        if (interval_nanoseconds != 0) {
          next_execution =
            now + rclcpp::Duration::from_nanoseconds(
                    interval_nanoseconds - (duration_nanoseconds % interval_nanoseconds));
        } else {
          next_execution = now;
        }
      }
    }
  };

  std::unordered_map<std::string, std::vector<Task>> tasks;

  std::atomic<bool> task_clear_requested = false;
};

#endif  // CONCEALER__AUTOWARE_STATE_DISPATCHER_HPP_
