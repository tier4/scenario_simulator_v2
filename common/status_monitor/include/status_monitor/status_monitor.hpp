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

#ifndef STATUS_MONITOR__STATUS_MONITOR_HPP_
#define STATUS_MONITOR__STATUS_MONITOR_HPP_

#include <chrono>
#include <thread>
#include <unordered_map>

namespace common
{
class StatusMonitor
{
  struct Status
  {
    std::chrono::high_resolution_clock::time_point last_access;
  };

  static inline std::unordered_map<std::thread::id, Status> statuses;

  static inline std::thread watchdog;

  static inline std::atomic_bool terminating;

public:
  explicit StatusMonitor();

  ~StatusMonitor();

  auto touch(std::thread::id id)
  {
    // TODO MUTEX LOCK
    statuses[id].last_access = std::chrono::high_resolution_clock::now();
  }
} static status_monitor;
}  // namespace common

#endif  // STATUS_MONITOR__STATUS_MONITOR_HPP_
