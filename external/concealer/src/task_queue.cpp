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

#include <chrono>
#include <concealer/task_queue.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>

namespace concealer
{
TaskQueue::TaskQueue()
: dispatcher(
    [this](auto notification) {
      using namespace std::literals::chrono_literals;
      while (rclcpp::ok() and notification.wait_for(1ms) == std::future_status::timeout) {
        if (not thunks.empty() and not thrown) {
          // NOTE: To ensure that the task to be queued is completed as expected is the responsibility of the side to create a task.
          std::thread(thunks.front()).join();
          thunks.pop();
        } else {
          std::this_thread::sleep_for(100ms);
        }
      }
    },
    std::move(notifier.get_future()))
{
}

TaskQueue::~TaskQueue()
{
  if (dispatcher.joinable()) {
    notifier.set_value();
    dispatcher.join();
  }
}

bool TaskQueue::exhausted() const noexcept { return thunks.empty(); }

void TaskQueue::rethrow() const
{
  if (thrown) {
    std::rethrow_exception(thrown);
  }
}
}  // namespace concealer
