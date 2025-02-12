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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_TRAJECTORY__FOLLOW_TRAJECTORY_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_TRAJECTORY__FOLLOW_TRAJECTORY_HPP_

#include <optional>
#include <traffic_simulator/behavior/follow_trajectory/follow_waypoint_controller.hpp>
#include <traffic_simulator/behavior/follow_trajectory/polyline_trajectory_positioner.hpp>
#include <traffic_simulator/behavior/follow_trajectory/validated_entity_status.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
/// @note side effects on polyline_trajectory
auto makeUpdatedEntityStatus(
  const ValidatedEntityStatus & validated_entity_status,
  traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
  const double matching_distance, const std::optional<double> target_speed, const double step_time,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<EntityStatus>;

}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__FOLLOW_TRAJECTORY__FOLLOW_TRAJECTORY_HPP_
