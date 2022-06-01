// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

namespace concealer
{
TaskQueue::TaskQueue()
: dispatcher([this] {
    try {
      while (rclcpp::ok() and not is_stop_requested.load(std::memory_order_acquire)) {
        using namespace std::literals::chrono_literals;
        std::unique_lock lk(thunks_mutex);
        if (not thunks.empty()) {
          // NOTE: To ensure that the task to be queued is completed as expected is the
          // responsibility of the side to create a task.
          auto f = std::move(thunks.front());
          thunks.pop();
          lk.unlock();

          f();
        } else {
          lk.unlock();
          std::this_thread::sleep_for(100ms);
        }
      }
    } catch (...) {
      thrown = std::current_exception();
      is_thrown.store(true, std::memory_order_release);
    }
  })
{
}

TaskQueue::~TaskQueue()
{
  if (dispatcher.joinable()) {
    is_stop_requested.store(true, std::memory_order_release);
    dispatcher.join();
  }
}

bool TaskQueue::exhausted() const noexcept { return thunks.empty(); }

void TaskQueue::rethrow() const
{
  if (is_thrown.load(std::memory_order_acquire)) {
    std::rethrow_exception(thrown);
  }
}
}  // namespace concealer
