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

#include <cerrno>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>
#include <unordered_map>
#include <utility>

namespace common
{
class StatusMonitor
{
  struct Status
  {
    const std::string name;

    std::chrono::high_resolution_clock::time_point last_access;

    using duration =
      decltype(std::chrono::high_resolution_clock::now() - std::chrono::high_resolution_clock::now());

    duration minimum_access_interval;

    duration maximum_access_interval;

    bool exited = false;

    template <typename Name>
    explicit Status(Name && name)
    : name(std::forward<decltype(name)>(name)),
      last_access(std::chrono::high_resolution_clock::now()),
      minimum_access_interval(duration::max()),
      maximum_access_interval(duration::min())
    {
    }

    auto since_last_access() const
    {
      return std::chrono::high_resolution_clock::now() - last_access;
    }

    auto good() const { return exited or since_last_access() < threshold; }

    explicit operator bool() const { return good(); }
  };

  static inline std::ofstream file;

  static inline std::unordered_map<std::thread::id, Status> statuses;

  static inline std::thread watchdog;

  static inline std::atomic_bool terminating;

  static inline std::chrono::seconds threshold = std::chrono::seconds(1);

public:
  explicit StatusMonitor();

  ~StatusMonitor();

  auto good() const
  {
    return std::all_of(std::begin(statuses), std::end(statuses), [](auto && id_and_status) {
      return std::get<1>(id_and_status).good();
    });
  }

  auto mark_as_exited()
  {
    if (auto iter = statuses.find(std::this_thread::get_id()); iter != std::end(statuses)) {
      std::get<1>(*iter).exited = true;
    }
  }

  auto name() const -> const std::string &;

  template <typename Name>
  auto touch(Name && name)
  {
    // TODO MUTEX LOCK
    if (auto iter = statuses.find(std::this_thread::get_id()); iter != std::end(statuses)) {
      [[maybe_unused]] auto && [id, status] = *iter;

      const auto now = std::chrono::high_resolution_clock::now();

      const auto this_access_interval = now - status.last_access;

      status.minimum_access_interval =
        std::min<std::decay_t<decltype(status.minimum_access_interval)>>(
          status.minimum_access_interval, this_access_interval);

      status.maximum_access_interval =
        std::max<std::decay_t<decltype(status.maximum_access_interval)>>(
          status.maximum_access_interval, this_access_interval);

      status.last_access = now;
    } else {
      statuses.emplace(std::this_thread::get_id(), std::forward<decltype(name)>(name));
    }
  }

  auto write() const -> void;
} static status_monitor;
}  // namespace common

#endif  // STATUS_MONITOR__STATUS_MONITOR_HPP_
