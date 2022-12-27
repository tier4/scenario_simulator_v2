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

#include <atomic>
#include <boost/lexical_cast.hpp>
#include <cstdint>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <status_monitor/status_monitor.hpp>

namespace common
{
static std::size_t count = 0;

StatusMonitor::StatusMonitor()
{
  if (not count++) {
    statuses = {};

    watchdog = std::thread([this]() {
      if (file.open("/tmp/" + name() + "_status"); not file.is_open()) {
        std::cout << "FILE OPEN FAILED!!!" << std::endl;
      }

      while (not terminating.load(std::memory_order_acquire)) {
        write();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      std::cout << "WATCHDOG " << std::this_thread::get_id() << " TERMINATED" << std::endl;
    });
  }
}

StatusMonitor::~StatusMonitor()
{
  mark_as_exited();

  if (not --count) {
    terminating.store(true, std::memory_order_release);
    watchdog.join();
    terminating.store(false, std::memory_order_release);

    file.close();
  }
}

auto StatusMonitor::name() const -> const std::string &
{
  static const std::string name =
#if _GNU_SOURCE
    std::filesystem::path(program_invocation_name).filename().string();
#else
    "thread_" + boost::lexical_cast<std::string>(std::this_thread::get_id());
#endif
  return name;
}

auto StatusMonitor::write() const -> void
{
  nlohmann::json json;

  json["details"] = nlohmann::json::array();

  if (not statuses.empty()) {
    for (auto && [id, status] : statuses) {
      nlohmann::json detail;

      detail["exited"] = status.exited;

      detail["good"] = status.good();

      detail["maximum_access_interval_ms"] =
        std::chrono::duration_cast<std::chrono::milliseconds>(status.maximum_access_interval)
          .count();

      detail["minimum_access_interval_ms"] =
        std::chrono::duration_cast<std::chrono::milliseconds>(status.minimum_access_interval)
          .count();

      detail["thread_id"] = boost::lexical_cast<std::string>(id);

      detail["name"] = status.name;

      detail["since_last_access_ms"] =
        std::chrono::duration_cast<std::chrono::milliseconds>(status.since_last_access()).count();

      json["details"].push_back(detail);
    }
  }

  json["name"] = name();

  json["good"] = good();

  json["unix_time"] = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::high_resolution_clock::now().time_since_epoch())
                        .count();

  file << json.dump() << std::endl;
}
}  // namespace common
