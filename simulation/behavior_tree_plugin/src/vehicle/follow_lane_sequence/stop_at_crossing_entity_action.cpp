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

#include <behavior_tree_plugin/vehicle/behavior_tree.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/stop_at_crossing_entity_action.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <utility>
#include <vector>

namespace entity_behavior
{
namespace vehicle
{
namespace follow_lane_sequence
{
StopAtCrossingEntityAction::StopAtCrossingEntityAction(
  const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config)
{
  in_stop_sequence_ = false;
}

const std::optional<traffic_simulator_msgs::msg::Obstacle>
StopAtCrossingEntityAction::calculateObstacle(const traffic_simulator_msgs::msg::WaypointsArray &)
{
  if (!distance_to_stop_target_) {
    return std::nullopt;
  }
  if (distance_to_stop_target_.value() < 0) {
    return std::nullopt;
  }
  if (distance_to_stop_target_.value() > trajectory->getLength()) {
    return std::nullopt;
  }
  traffic_simulator_msgs::msg::Obstacle obstacle;
  obstacle.type = obstacle.ENTITY;
  obstacle.s = distance_to_stop_target_.value();
  return obstacle;
}

const traffic_simulator_msgs::msg::WaypointsArray StopAtCrossingEntityAction::calculateWaypoints()
{
  if (!canonicalized_entity_status->laneMatchingSucceed()) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (canonicalized_entity_status->getTwist().linear.x >= 0) {
    traffic_simulator_msgs::msg::WaypointsArray waypoints;
    const auto lanelet_pose = canonicalized_entity_status->getLaneletPose();
    waypoints.waypoints = reference_trajectory->getTrajectory(
      lanelet_pose.s, lanelet_pose.s + getHorizon(), 1.0, lanelet_pose.offset);
    trajectory = std::make_unique<math::geometry::CatmullRomSubspline>(
      reference_trajectory, lanelet_pose.s, lanelet_pose.s + getHorizon());
    return waypoints;
  } else {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
}

std::optional<double> StopAtCrossingEntityAction::calculateTargetSpeed(double current_velocity)
{
  if (!distance_to_stop_target_) {
    return std::nullopt;
  }
  double rest_distance =
    distance_to_stop_target_.value() - (vehicle_parameters.bounding_box.dimensions.x * 0.5 + 1);
  if (rest_distance < calculateStopDistance(behavior_parameter.dynamic_constraints)) {
    return 0;
  }
  return current_velocity;
}

BT::NodeStatus StopAtCrossingEntityAction::tick()
{
  getBlackBoardValues();
  if (
    request != traffic_simulator::behavior::Request::NONE &&
    request != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    in_stop_sequence_ = false;
    return BT::NodeStatus::FAILURE;
  }
  if (!canonicalized_entity_status->laneMatchingSucceed()) {
    in_stop_sequence_ = false;
    return BT::NodeStatus::FAILURE;
  }
  if (!behavior_parameter.see_around) {
    in_stop_sequence_ = false;
    return BT::NodeStatus::FAILURE;
  }
  if (getRightOfWayEntities(route_lanelets).size() != 0) {
    in_stop_sequence_ = false;
    return BT::NodeStatus::FAILURE;
  }
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (trajectory == nullptr) {
    return BT::NodeStatus::FAILURE;
  }
  distance_to_stop_target_ = getDistanceToConflictingEntity(route_lanelets, *trajectory);
  auto distance_to_stopline = hdmap_utils->getDistanceToStopLine(route_lanelets, *trajectory);
  const auto distance_to_front_entity = getDistanceToFrontEntity(*trajectory);
  if (!distance_to_stop_target_) {
    in_stop_sequence_ = false;
    return BT::NodeStatus::FAILURE;
  }
  if (distance_to_front_entity) {
    if (distance_to_front_entity.value() <= distance_to_stop_target_.value()) {
      in_stop_sequence_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }
  if (distance_to_stopline) {
    if (distance_to_stopline.value() <= distance_to_stop_target_.value()) {
      in_stop_sequence_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }
  std::optional<double> target_linear_speed;
  if (distance_to_stop_target_) {
    target_linear_speed = calculateTargetSpeed(canonicalized_entity_status->getTwist().linear.x);
  } else {
    target_linear_speed = std::nullopt;
  }
  if (!distance_to_stop_target_) {
    setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(0));
    setOutput("waypoints", waypoints);
    setOutput("obstacle", calculateObstacle(waypoints));
    in_stop_sequence_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  if (target_speed) {
    if (target_speed.value() > target_linear_speed.value()) {
      target_speed = target_linear_speed.value();
    }
  } else {
    target_speed = target_linear_speed.value();
  }
  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed.value()));
  setOutput("waypoints", waypoints);
  setOutput("obstacle", calculateObstacle(waypoints));
  in_stop_sequence_ = true;
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
