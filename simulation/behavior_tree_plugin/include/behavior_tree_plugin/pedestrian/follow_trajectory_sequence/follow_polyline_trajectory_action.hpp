// Copyright 2023 TIER IV, Inc. All rights reserved.
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

#ifndef BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__FOLLOW_POLYLINE_TRAJECTORY_ACTION_HPP_
#define BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__FOLLOW_POLYLINE_TRAJECTORY_ACTION_HPP_

#include <behavior_tree_plugin/pedestrian/pedestrian_action_node.hpp>

namespace entity_behavior
{
namespace pedestrian
{
struct FollowPolylineTrajectoryAction : public PedestrianActionNode
{
  std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> polyline_trajectory;

  using PedestrianActionNode::PedestrianActionNode;

  auto calculateWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray;

  auto calculateObstacle(const traffic_simulator_msgs::msg::WaypointsArray &)
    -> const std::optional<traffic_simulator_msgs::msg::Obstacle>;

  static auto providedPorts() -> BT::PortsList;

  auto tick() -> BT::NodeStatus override;
};
}  // namespace pedestrian
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__FOLLOW_POLYLINE_TRAJECTORY_ACTION_HPP_
