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

#include <behavior_tree_plugin/vehicle/follow_trajectory_sequence/follow_clothoid_trajectory_action.hpp>

namespace entity_behavior
{
namespace vehicle
{
auto FollowClothoidTrajectoryAction::calculateWaypoints()
  -> const traffic_simulator_msgs::msg::WaypointsArray
{
  return traffic_simulator_msgs::msg::WaypointsArray();
}

auto FollowClothoidTrajectoryAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
  -> const std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return std::nullopt;
}

auto FollowClothoidTrajectoryAction::tick() -> BT::NodeStatus { return BT::NodeStatus::FAILURE; }
}  // namespace vehicle
}  // namespace entity_behavior
