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

/// @todo: pzyskowski : use conditional variables to wait for new messages instead of sleep_for
namespace concealer
{
TaskQueue::TaskQueue()
: dispatcher([this] {
    try {
      while (rclcpp::ok() and not finalized.load(std::memory_order_acquire)) {
        if (not empty()) {
          auto thunk = front();
          thunk();
          pop();
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    } catch (...) {
      thrown = std::current_exception();
    }
  })
{
}

TaskQueue::~TaskQueue()
{
  if (dispatcher.joinable()) {
    finalized.store(true, std::memory_order_release);
    dispatcher.join();
  }
}

auto TaskQueue::empty() const -> bool
{
  auto lock = std::unique_lock(thunks_mutex);
  return thunks.empty();
}

auto TaskQueue::front() const -> Thunk
{
  auto lock = std::unique_lock(thunks_mutex);
  return thunks.front();
}

auto TaskQueue::pop() -> void
{
  auto lock = std::unique_lock(thunks_mutex);
  thunks.pop();
}

auto TaskQueue::rethrow() const -> void
{
  if (thrown) {
    std::rethrow_exception(thrown);
  }
}
}  // namespace concealer
