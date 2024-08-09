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

#include <algorithm>
#include <behavior_tree_plugin/vehicle/behavior_tree.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/follow_front_entity_action.hpp>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <vector>

namespace entity_behavior
{
namespace vehicle
{
namespace follow_lane_sequence
{
FollowFrontEntityAction::FollowFrontEntityAction(
  const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config)
{
}

const std::optional<traffic_simulator_msgs::msg::Obstacle>
FollowFrontEntityAction::calculateObstacle(const traffic_simulator_msgs::msg::WaypointsArray &)
{
  if (!distance_to_front_entity_) {
    return std::nullopt;
  }
  if (distance_to_front_entity_.value() < 0) {
    return std::nullopt;
  }
  if (distance_to_front_entity_.value() > trajectory->getLength()) {
    return std::nullopt;
  }
  traffic_simulator_msgs::msg::Obstacle obstacle;
  obstacle.type = obstacle.ENTITY;
  obstacle.s = distance_to_front_entity_.value();
  return obstacle;
}

const traffic_simulator_msgs::msg::WaypointsArray FollowFrontEntityAction::calculateWaypoints()
{
  if (!canonicalized_entity_status->laneMatchingSucceed()) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (canonicalized_entity_status->getTwist().linear.x >= 0) {
    traffic_simulator_msgs::msg::WaypointsArray waypoints;
    double horizon = getHorizon();
    const auto lanelet_pose = canonicalized_entity_status->getLaneletPose();
    waypoints.waypoints = reference_trajectory->getTrajectory(
      lanelet_pose.s, lanelet_pose.s + horizon, 1.0, lanelet_pose.offset);
    trajectory = std::make_unique<math::geometry::CatmullRomSubspline>(
      reference_trajectory, lanelet_pose.s, lanelet_pose.s + horizon);
    return waypoints;
  } else {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
}

BT::NodeStatus FollowFrontEntityAction::tick()
{
  getBlackBoardValues();
  if (
    request != traffic_simulator::behavior::Request::NONE &&
    request != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return BT::NodeStatus::FAILURE;
  }
  if (getRightOfWayEntities(route_lanelets).size() != 0) {
    return BT::NodeStatus::FAILURE;
  }
  if (!behavior_parameter.see_around) {
    return BT::NodeStatus::FAILURE;
  }
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (trajectory == nullptr) {
    return BT::NodeStatus::FAILURE;
  }
  auto distance_to_stopline = hdmap_utils->getDistanceToStopLine(route_lanelets, *trajectory);
  auto distance_to_conflicting_entity = getDistanceToConflictingEntity(route_lanelets, *trajectory);
  const auto front_entity_name = getFrontEntityName(*trajectory);
  if (!front_entity_name) {
    return BT::NodeStatus::FAILURE;
  }
  distance_to_front_entity_ =
    getDistanceToTargetEntityPolygon(*trajectory, front_entity_name.value());
  if (!distance_to_front_entity_) {
    return BT::NodeStatus::FAILURE;
  }
  if (distance_to_conflicting_entity) {
    if (distance_to_front_entity_.value() > distance_to_conflicting_entity.value()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  if (distance_to_stopline) {
    if (distance_to_front_entity_.value() > distance_to_stopline.value()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  const auto & front_entity_status = getEntityStatus(front_entity_name.value());
  if (!target_speed) {
    target_speed = hdmap_utils->getSpeedLimit(route_lanelets);
  }
  const double front_entity_linear_velocity = front_entity_status.getTwist().linear.x;
  if (target_speed.value() <= front_entity_linear_velocity) {
    setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed.value()));
    const auto obstacle = calculateObstacle(waypoints);
    setOutput("waypoints", waypoints);
    setOutput("obstacle", obstacle);
    return BT::NodeStatus::RUNNING;
  }
  if (
    distance_to_front_entity_.value() >=
    (calculateStopDistance(behavior_parameter.dynamic_constraints) +
     vehicle_parameters.bounding_box.dimensions.x + 5)) {
    setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(front_entity_linear_velocity + 2));
    setOutput("waypoints", waypoints);
    setOutput("obstacle", calculateObstacle(waypoints));
    return BT::NodeStatus::RUNNING;
  } else if (
    distance_to_front_entity_.value() <=
    calculateStopDistance(behavior_parameter.dynamic_constraints)) {
    setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(front_entity_linear_velocity - 2));
    setOutput("waypoints", waypoints);
    setOutput("obstacle", calculateObstacle(waypoints));
    return BT::NodeStatus::RUNNING;
  } else {
    setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(front_entity_linear_velocity));
    setOutput("waypoints", waypoints);
    setOutput("obstacle", calculateObstacle(waypoints));
    return BT::NodeStatus::RUNNING;
  }
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
