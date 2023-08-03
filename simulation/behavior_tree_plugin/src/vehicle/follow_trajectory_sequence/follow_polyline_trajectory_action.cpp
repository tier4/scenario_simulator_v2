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

#include <behavior_tree_plugin/vehicle/follow_trajectory_sequence/follow_polyline_trajectory_action.hpp>

namespace entity_behavior
{
namespace vehicle
{
auto FollowPolylineTrajectoryAction::calculateWaypoints()
  -> const traffic_simulator_msgs::msg::WaypointsArray
{
  auto waypoints = traffic_simulator_msgs::msg::WaypointsArray();
  waypoints.waypoints.push_back(entity_status.pose.position);
  for (const auto & vertex : polyline_trajectory->shape.vertices) {
    waypoints.waypoints.push_back(vertex.position.position);
  }
  return waypoints;
}

auto FollowPolylineTrajectoryAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
  -> const std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  /**
     Obstacle avoidance is not implemented for this action.

     @todo If you implement obstacle avoidance for this action, implement this
     virtual function to return the location of any obstacles blocking the path
     of this action's actor. However, this virtual function is currently used
     only for the visualization of obstacle information, so the obstacle
     avoidance algorithm does not necessarily need to use this virtual
     function.
  */
  return std::nullopt;
}

auto FollowPolylineTrajectoryAction::providedPorts() -> BT::PortsList
{
  auto ports = VehicleActionNode::providedPorts();
  ports.emplace(BT::InputPort<decltype(polyline_trajectory)>("polyline_trajectory"));
  ports.emplace(BT::InputPort<decltype(target_speed)>("target_speed"));
  return ports;
}

auto FollowPolylineTrajectoryAction::tick() -> BT::NodeStatus
{
  if (!trajectory_follower) {
    trajectory_follower = std::make_unique<traffic_simulator::behavior::PositionModePolylineTrajectoryFollower>();
  }

  if (getBlackBoardValues();
      request != traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY or
      not getInput<decltype(polyline_trajectory)>("polyline_trajectory", polyline_trajectory) or
      not getInput<decltype(target_speed)>("target_speed", target_speed) or
      not polyline_trajectory) {
    trajectory_follower.reset();
    return BT::NodeStatus::FAILURE;
  } else if (trajectory_follower->setParameters(entity_status, behavior_parameter, step_time);
             const auto updated_status =
               trajectory_follower->followTrajectory(polyline_trajectory)) {
    setOutput("updated_status", *updated_status);
    setOutput("waypoints", calculateWaypoints());
    setOutput("obstacle", calculateObstacle(calculateWaypoints()));
    return BT::NodeStatus::RUNNING;
  } else {
    trajectory_follower.reset();
    return BT::NodeStatus::SUCCESS;
  }
}
}  // namespace vehicle
}  // namespace entity_behavior
