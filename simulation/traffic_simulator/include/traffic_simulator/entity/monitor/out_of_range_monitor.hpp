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

#ifndef TRAFFIC_SIMULATOR__ENTITY__MONITOR__OUT_OF_RANGE_MONITOR_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__MONITOR__OUT_OF_RANGE_MONITOR_HPP_

#include <optional>
#include <rclcpp/qos.hpp>
#include <string>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <traffic_simulator/entity/entity_base.hpp>

namespace traffic_simulator::entity
{
class OutOfRangeMonitor
{
  using JerkMessageType = tier4_debug_msgs::msg::Float32Stamped;

public:
  /**
   * @brief Construct a new Out Of Range Monitor object
   * @param target_entity Name of target entity
   * @param min_velocity If the velocity of the target entity go below this value, this metrics becomes failure state.
   * @param max_velocity If the velocity of the target entity overs this value, this metrics becomes failure state.
   * @param min_acceleration If the acceleration of the target entity go below this value, this metrics becomes failure state.
   * @param max_acceleration If the acceleration of the target entity overs this value, this metrics becomes failure state.
   * @param min_jerk If the jerk of the target entity go below this value, this metrics becomes failure state.
   * @param max_jerk If the jerk of the target entity overs this value, this metrics becomes failure state.
   */
  OutOfRangeMonitor(
    EntityBase & entity, double min_velocity, double max_velocity, double min_acceleration,
    double max_acceleration, double min_jerk, double max_jerk,
    std::optional<std::string> jerk_topic = std::nullopt);

  auto operator()(double) -> bool;

private:
  const double min_velocity_;
  const double max_velocity_;
  const double min_acceleration_;
  const double max_acceleration_;
  const double min_jerk_;
  const double max_jerk_;
  const std::optional<std::string> jerk_topic_;

  EntityBase & entity_;
  double linear_jerk_ = 0;

  rclcpp::Subscription<JerkMessageType>::SharedPtr jerk_callback_ptr_;
};
}  // namespace traffic_simulator::entity

#endif  // TRAFFIC_SIMULATOR__ENTITY__MONITOR__OUT_OF_RANGE_MONITOR_HPP_
