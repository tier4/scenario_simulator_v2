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

#ifndef TRAFFIC_SIMULATOR__HELPER__STOP_WATCH_HPP_
#define TRAFFIC_SIMULATOR__HELPER__STOP_WATCH_HPP_

#include <chrono>
#include <iostream>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace traffic_simulator
{
namespace helper
{
template <typename T>
class StopWatch
{
public:
  explicit StopWatch(const std::string & name, bool verbose = false, bool autostart = true)
  : name(name), verbose(verbose)
  {
    if (autostart) {
      start();
    }
  }

  void start()
  {
    end_time = std::nullopt;
    start_time = std::chrono::system_clock::now();
  }
  void stop() { end_time = std::chrono::system_clock::now(); }
  void print()
  {
    if (verbose) {
      if (start_time && end_time) {
        double elapsed =
          std::chrono::duration_cast<T>(end_time.value() - start_time.value()).count();
        if (typeid(T) == typeid(std::chrono::microseconds)) {
          RCLCPP_INFO_STREAM(
            rclcpp::get_logger(name),
            "elapsed time in stop watch " << name << " : " << elapsed << " microseconds");
        }
        if (typeid(T) == typeid(std::chrono::milliseconds)) {
          RCLCPP_INFO_STREAM(
            rclcpp::get_logger(name),
            "elapsed time in stop watch " << name << " : " << elapsed << " milliseconds");
        }
        if (typeid(T) == typeid(std::chrono::seconds)) {
          RCLCPP_INFO_STREAM(
            rclcpp::get_logger(name),
            "elapsed time in stop watch " << name << " : " << elapsed << " seconds");
        }
      }
    } else {
      if (start_time) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(name), "Stop watch : " << name << " did not started.");
      }
      if (end_time) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(name), "Stop watch : " << name << " did not stopped.");
      }
    }
  }

public:
  const std::string name;
  const bool verbose;

private:
  std::optional<std::chrono::system_clock::time_point> start_time, end_time;
};
}  // namespace helper
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__HELPER__STOP_WATCH_HPP_
