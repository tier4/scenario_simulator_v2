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

#include <string>
#include <traffic_simulator/entity/monitor/out_of_range_monitor.hpp>

namespace traffic_simulator::entity
{
OutOfRangeMonitor::OutOfRangeMonitor(
  EntityBase & entity, double min_velocity, double max_velocity, double min_acceleration,
  double max_acceleration, double min_jerk, double max_jerk, std::optional<std::string> jerk_topic)
: min_velocity_(min_velocity),
  max_velocity_(max_velocity),
  min_acceleration_(min_acceleration),
  max_acceleration_(max_acceleration),
  min_jerk_(min_jerk),
  max_jerk_(max_jerk),
  jerk_topic_(std::move(jerk_topic)),
  entity_(entity)
{
}

auto OutOfRangeMonitor::operator()(double) -> bool
{
  const auto status = entity_.getStatus();

  auto linear_velocity = status.action_status.twist.linear.x;
  auto linear_acceleration = status.action_status.accel.linear.x;

  if (min_velocity_ > linear_velocity || linear_velocity > max_velocity_) {
    throw SPECIFICATION_VIOLATION(
      "current velocity (which is ", linear_velocity, ") is out of range (which is [",
      min_velocity_, ", ", max_velocity_, "])");
  }

  if (min_acceleration_ > linear_acceleration || linear_acceleration > max_acceleration_) {
    throw SPECIFICATION_VIOLATION(
      "current acceleration (which is ", linear_acceleration, ") is out of range (which is [",
      min_acceleration_, ", ", max_acceleration_, "])");
  }

  // TODO: get jerk via callback
  if (not jerk_callback_ptr_) {
    linear_jerk_ = entity_.getLinearJerk();
  }

  if (min_jerk_ > linear_jerk_ || linear_jerk_ > max_jerk_) {
    throw SPECIFICATION_VIOLATION(
      "current jerk (which is ", linear_jerk_, ") is out of range (which is [", min_jerk_, ", ",
      max_jerk_, "])");
  }

  return true;
}

}  // namespace traffic_simulator::entity
