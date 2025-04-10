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

#ifndef TRAFFIC_SIMULATOR__TEST__TRAFFIC_LIGHTS__HELPER_HPP_
#define TRAFFIC_SIMULATOR__TEST__TRAFFIC_LIGHTS__HELPER_HPP_

#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>

/// Helper functions
// clang-format off
inline auto stateFromColor(const std::string & color)   -> std::string { return color + " solidOn circle"; }
inline auto stateFromStatus(const std::string & status) -> std::string { return "green " + status + " circle"; }
inline auto stateFromShape(const std::string & shape)   -> std::string { return "green solidOn " + shape; }
// clang-format on

/// Returns time in nanoseconds
inline auto getTime(const builtin_interfaces::msg::Time & time) -> int
{
  static constexpr int nanosecond_multiplier = static_cast<int>(1e+9);
  return static_cast<int>(time.sec) * nanosecond_multiplier + static_cast<int>(time.nanosec);
}

/// Returns time in nanoseconds
inline auto getTime(const std_msgs::msg::Header & header) -> int { return getTime(header.stamp); }

/// Returns time in nanoseconds
inline auto getTime(const rclcpp::Time & time) -> int
{
  return static_cast<int>(time.nanoseconds());
}

#endif  // TRAFFIC_SIMULATOR__TEST__TRAFFIC_LIGHTS__HELPER_HPP_
