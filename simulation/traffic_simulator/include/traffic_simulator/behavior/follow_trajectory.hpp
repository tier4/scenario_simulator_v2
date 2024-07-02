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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_TRAJECTORY_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_TRAJECTORY_HPP_

#include <optional>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
auto makeUpdatedStatus(
  const traffic_simulator_msgs::msg::EntityStatus &,
  traffic_simulator_msgs::msg::PolylineTrajectory &,
  const traffic_simulator_msgs::msg::BehaviorParameter &,
  const std::shared_ptr<hdmap_utils::HdMapUtils> &, double, double,
  std::optional<double> target_speed = std::nullopt) -> std::optional<EntityStatus>;
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_TRAJECTORY_HPP_
