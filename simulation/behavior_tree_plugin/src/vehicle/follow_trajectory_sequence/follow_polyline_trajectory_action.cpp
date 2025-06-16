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
  waypoints.waypoints.push_back(canonicalized_entity_status_->getMapPose().position);
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
  ports.emplace(BT::InputPort<decltype(target_speed_)>("target_speed"));
  return ports;
}

bool FollowPolylineTrajectoryAction::checkPreconditions()
{
  if (
    request_ != traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY or
    not getInput<decltype(polyline_trajectory)>("polyline_trajectory", polyline_trajectory) or
    not getInput<decltype(target_speed_)>("target_speed", target_speed_) or
    not polyline_trajectory) {
    return false;
  } else if (std::isnan(canonicalized_entity_status_->getTime())) {
    THROW_SIMULATION_ERROR(
      "Time in canonicalized_entity_status is NaN - FollowTrajectoryAction does not support such "
      "case.");
  } else {
    return true;
  }
}

BT::NodeStatus FollowPolylineTrajectoryAction::doAction()
{
  auto getTargetSpeed = [&]() -> double {
    if (target_speed_.has_value()) {
      return target_speed_.value();
    } else {
      return canonicalized_entity_status_->getTwist().linear.x;
    }
  };

  if (
    const auto entity_status_updated = traffic_simulator::follow_trajectory::makeUpdatedStatus(
      static_cast<traffic_simulator::EntityStatus>(*canonicalized_entity_status_),
      *polyline_trajectory, behavior_parameter_, hdmap_utils_, step_time_,
      default_matching_distance_for_lanelet_pose_calculation_, getTargetSpeed())) {
    setCanonicalizedEntityStatus(entity_status_updated.value());
    setOutput("waypoints", calculateWaypoints());
    setOutput("obstacle", calculateObstacle(calculateWaypoints()));
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}
}  // namespace vehicle
}  // namespace entity_behavior
