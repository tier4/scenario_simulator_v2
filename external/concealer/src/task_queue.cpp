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
#include <iostream>
#include <concealer/task_queue.hpp>
#include <rclcpp/rclcpp.hpp>

/// @todo: pzyskowski : use conditional variables to wait for new messages instead of sleep_for
namespace concealer
{
TaskQueue::TaskQueue()
: dispatcher([this] {
    std::cerr << "[WARN][DEBUG/concealer::FieldOperatorApplication::TaskQueue] TaskQueue dispatcher thread started" << std::endl;
    std::cerr << "[WARN][DEBUG/concealer::FieldOperatorApplication::TaskQueue] rclcpp::ok() = " << (rclcpp::ok() ? "true" : "false") << std::endl;
    std::cerr << "[WARN][DEBUG/concealer::FieldOperatorApplication::TaskQueue] finalized = " << finalized.load(std::memory_order_acquire) << std::endl;
    if (rclcpp::ok()) {
      RCLCPP_WARN(rclcpp::get_logger("DEBUG/concealer::FieldOperatorApplication::TaskQueue"), "TaskQueue dispatcher thread started");
      RCLCPP_WARN(rclcpp::get_logger("DEBUG/concealer::FieldOperatorApplication::TaskQueue"), "rclcpp::ok() = true");
      RCLCPP_WARN(rclcpp::get_logger("DEBUG/concealer::FieldOperatorApplication::TaskQueue"), "finalized = %s", finalized.load() ? "true" : "false");
    }
    try {
      while (rclcpp::ok() and not finalized.load(std::memory_order_acquire)) {
        if (not empty()) {
          RCLCPP_WARN(rclcpp::get_logger("DEBUG/concealer::FieldOperatorApplication::TaskQueue"), "Call thunk");
          auto thunk = front();
          thunk();
          pop();
          RCLCPP_WARN(rclcpp::get_logger("DEBUG/concealer::FieldOperatorApplication::TaskQueue"), "Thunk task completed successfully");
        } else {
          static int empty_count = 0;
          if (++empty_count % 30 == 0) {  // Log every 3 seconds (30 * 100ms)
            RCLCPP_WARN(rclcpp::get_logger("DEBUG/concealer::FieldOperatorApplication::TaskQueue"), "Queue empty, waiting... (count: %d)", empty_count);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    } catch (...) {
      std::cerr << "[WARN][DEBUG/concealer::FieldOperatorApplication::TaskQueue] Exception" << std::endl;
      if (rclcpp::ok()) {
        RCLCPP_WARN(rclcpp::get_logger("DEBUG/concealer::FieldOperatorApplication::TaskQueue"), "Exception");
      }
      thrown = std::current_exception();
    }
    std::cerr << "[WARN][DEBUG/concealer::FieldOperatorApplication::TaskQueue] TaskQueue dispatcher thread ended" << std::endl;
    std::cerr << "[WARN][DEBUG/concealer::FieldOperatorApplication::TaskQueue] Final rclcpp::ok() = " << (rclcpp::ok() ? "true" : "false") << std::endl;
    std::cerr << "[WARN][DEBUG/concealer::FieldOperatorApplication::TaskQueue] Final finalized = " << finalized.load(std::memory_order_acquire) << std::endl;
    if (rclcpp::ok()) {
      RCLCPP_WARN(rclcpp::get_logger("DEBUG/concealer::FieldOperatorApplication::TaskQueue"), "TaskQueue dispatcher thread ended");
      RCLCPP_WARN(rclcpp::get_logger("DEBUG/concealer::FieldOperatorApplication::TaskQueue"), "Final rclcpp::ok() = true");
      RCLCPP_WARN(rclcpp::get_logger("DEBUG/concealer::FieldOperatorApplication::TaskQueue"), "Final finalized = %s", finalized.load(std::memory_order_acquire) ? "true" : "false");
    } 
  })
{
  std::cerr << "[WARN][DEBUG/concealer::FieldOperatorApplication::TaskQueue] TaskQueue constructor finished, rclcpp::ok() = " << (rclcpp::ok() ? "true" : "false") << std::endl;
  if (rclcpp::ok()) {
    RCLCPP_WARN(rclcpp::get_logger("DEBUG/concealer::FieldOperatorApplication::TaskQueue]"), "TaskQueue constructor finished, rclcpp::ok() = true");
  }
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
