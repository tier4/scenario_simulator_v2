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
static auto name() -> const std::string &
{
  static const std::string name =
#if _GNU_SOURCE
    std::filesystem::path(program_invocation_name).filename().string();
#else
    "thread_" + boost::lexical_cast<std::string>(std::this_thread::get_id());
#endif
  return name;
}

static std::size_t count = 0;

StatusMonitor::StatusMonitor()
{
  if (not count++) {
    statuses = {};

    watchdog = std::thread([this]() {
      /*
         When a file open fails, the system expects the upper-level system to
         determine that the system is in an abnormal state because the file
         that should have been output does not exist.
      */
      file.open("/tmp/" + name() + "_status.json");

      while (not terminating.load(std::memory_order_acquire)) {
        write();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });
  }
}

StatusMonitor::~StatusMonitor()
{
  auto lock = std::scoped_lock<std::mutex>(mutex);

  auto mark_as_exited = [this]() {
    if (auto iter = statuses.find(std::this_thread::get_id()); iter != std::end(statuses)) {
      std::get<1>(*iter).exited = true;
    }
  };

  mark_as_exited();

  if (not --count) {
    terminating.store(true, std::memory_order_release);
    watchdog.join();
    terminating.store(false, std::memory_order_release);

    file.close();
  }
}

auto StatusMonitor::write() const -> void
{
  auto lock = std::scoped_lock<std::mutex>(mutex);

  auto good = []() {
    return std::all_of(std::begin(statuses), std::end(statuses), [](auto && id_and_status) {
      return std::get<1>(id_and_status).good();
    });
  };

  nlohmann::json json;

  json["threads"] = nlohmann::json::array();

  if (not statuses.empty()) {
    for (auto && [id, status] : statuses) {
      nlohmann::json thread;

      thread["good"] = status.good();

      thread["maximum_access_interval_ms"] =
        std::chrono::duration_cast<std::chrono::milliseconds>(status.maximum_access_interval)
          .count();

      thread["minimum_access_interval_ms"] =
        std::chrono::duration_cast<std::chrono::milliseconds>(status.minimum_access_interval)
          .count();

      thread["thread_id"] = boost::lexical_cast<std::string>(id);

      thread["name"] = status.name;

      thread["since_last_access_ms"] =
        std::chrono::duration_cast<std::chrono::milliseconds>(status.since_last_access()).count();

      json["threads"].push_back(thread);

      if (not status.good()) {
        std::cout << status.name << " of " << name() << " is probably unresponsive." << std::endl;
      }
    }
  }

  json["good"] = good();

  json["name"] = name();

  json["threshold"] = threshold.count();

  json["unix_time"] = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::high_resolution_clock::now().time_since_epoch())
                        .count();

  file << json.dump() << std::endl;
}
}  // namespace common
