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
  auto current_thread_id = std::this_thread::get_id();
  std::cout << "[StatusMonitor] Constructor called for thread: " << boost::lexical_cast<std::string>(current_thread_id) << std::endl;
  std::cout << "[StatusMonitor] Reference count before increment: " << count << std::endl;
  
  if (not count++) {
    std::cout << "[StatusMonitor] First instance - initializing watchdog" << std::endl;
    statuses = {};

    watchdog = std::thread([this]() {
      /*
         When a file open fails, the system expects the upper-level system to
         determine that the system is in an abnormal state because the file
         that should have been output does not exist.
      */
      std::string filename = "/tmp/" + name() + "_status.json";
      std::cout << "[StatusMonitor] Watchdog thread started, monitoring file: " << filename << std::endl;
      file.open(filename);
      
      if (!file.is_open()) {
        std::cout << "[StatusMonitor] Failed to open status file: " << filename << std::endl;
      } else {
        std::cout << "[StatusMonitor] Status file opened successfully: " << filename << std::endl;
      }

      size_t iteration = 0;
      while (not terminating.load(std::memory_order_acquire)) {
        iteration++;
        auto start_time = std::chrono::high_resolution_clock::now();
        
        write();
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto write_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        
        if (iteration % 10 == 0) {  // Log every 10 iterations to avoid spam
          std::cout << "[StatusMonitor] Watchdog iteration " << iteration 
                    << ", write() took " << write_duration << "ms" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      
      std::cout << "[StatusMonitor] Watchdog thread exiting after " << iteration << " iterations" << std::endl;
    });
  }
}

StatusMonitor::~StatusMonitor()
{
  auto current_thread_id = std::this_thread::get_id();
  std::cout << "[StatusMonitor] Destructor called for thread: " << boost::lexical_cast<std::string>(current_thread_id) << std::endl;
  
  auto lock = std::scoped_lock<std::mutex>(mutex);

  auto mark_as_exited = [current_thread_id]() {
    if (auto iter = statuses.find(current_thread_id); iter != std::end(statuses)) {
      std::cout << "[StatusMonitor] Marking thread as exited: " << std::get<1>(*iter).name 
                << " (ID: " << boost::lexical_cast<std::string>(current_thread_id) << ")" << std::endl;
      std::get<1>(*iter).exited = true;
    } else {
      std::cout << "[StatusMonitor] Thread not found in statuses: " << boost::lexical_cast<std::string>(current_thread_id) << std::endl;
    }
  };

  mark_as_exited();

  std::cout << "[StatusMonitor] Reference count before decrement: " << count << std::endl;
  
  if (not --count) {
    std::cout << "[StatusMonitor] Last instance - initiating watchdog shutdown" << std::endl;
    std::cout << "[StatusMonitor] Setting terminating flag to true" << std::endl;
    terminating.store(true, std::memory_order_release);
    
    std::cout << "[StatusMonitor] Waiting for watchdog thread to join..." << std::endl;
    watchdog.join();
    std::cout << "[StatusMonitor] Watchdog thread joined successfully" << std::endl;
    
    terminating.store(false, std::memory_order_release);
    std::cout << "[StatusMonitor] Terminating flag reset to false" << std::endl;

    file.close();
    std::cout << "[StatusMonitor] Status file closed" << std::endl;
  } else {
    std::cout << "[StatusMonitor] Reference count after decrement: " << count << std::endl;
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
        auto now = std::chrono::high_resolution_clock::now();
        auto now_unix = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        auto last_access_unix = std::chrono::duration_cast<std::chrono::milliseconds>(status.last_access.time_since_epoch()).count();
        auto since_last_ms = std::chrono::duration_cast<std::chrono::milliseconds>(status.since_last_access()).count();
        auto threshold_ms = std::chrono::duration_cast<std::chrono::milliseconds>(threshold).count();
        
        std::cout << "[StatusMonitor] " << status.name << " of " << name() << " is probably unresponsive." << std::endl;
        std::cout << "[StatusMonitor]   Thread ID: " << boost::lexical_cast<std::string>(id) << std::endl;
        std::cout << "[StatusMonitor]   Thread exited flag: " << (status.exited ? "true" : "false") << std::endl;
        std::cout << "[StatusMonitor]   Current time (unix ms): " << now_unix << std::endl;
        std::cout << "[StatusMonitor]   Last access time (unix ms): " << last_access_unix << std::endl;
        std::cout << "[StatusMonitor]   Time since last access (ms): " << since_last_ms << std::endl;
        std::cout << "[StatusMonitor]   Threshold (ms): " << threshold_ms << std::endl;
        std::cout << "[StatusMonitor]   Min access interval (ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(status.minimum_access_interval).count() << std::endl;
        std::cout << "[StatusMonitor]   Max access interval (ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(status.maximum_access_interval).count() << std::endl;
        std::cout << "[StatusMonitor]   System terminating flag: " << (terminating.load(std::memory_order_acquire) ? "true" : "false") << std::endl;
        std::cout << "[StatusMonitor]   Total monitored threads: " << statuses.size() << std::endl;
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
