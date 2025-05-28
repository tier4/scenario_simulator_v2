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
#include <traffic_simulator/utils/route.hpp>
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
  if (!canonicalized_entity_status_->isInLanelet()) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (canonicalized_entity_status_->getTwist().linear.x >= 0) {
    traffic_simulator_msgs::msg::WaypointsArray waypoints;
    const auto lanelet_pose = canonicalized_entity_status_->getLaneletPose();
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
  if (rest_distance < calculateStopDistance(behavior_parameter_.dynamic_constraints)) {
    return 0;
  }
  return current_velocity;
}

BT::NodeStatus StopAtCrossingEntityAction::tick()
{
  getBlackBoardValues();
  if (
    request_ != traffic_simulator::behavior::Request::NONE &&
    request_ != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return BT::NodeStatus::FAILURE;
  }
  if (!canonicalized_entity_status_->isInLanelet()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!behavior_parameter_.see_around) {
    return BT::NodeStatus::FAILURE;
  }
  if (traffic_simulator::route::isNeedToRightOfWay(
        route_lanelets_, getOtherEntitiesCanonicalizedLaneletPoses())) {
    return BT::NodeStatus::FAILURE;
  }
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (trajectory == nullptr) {
    return BT::NodeStatus::FAILURE;
  }
  std::optional<double> target_linear_speed;

  const auto distance_to_front_entity = getDistanceToFrontEntity(*trajectory);
  distance_to_stop_target_ = traffic_simulator::distance::distanceToNearestConflictingPose(
    route_lanelets_, *trajectory, *canonicalized_entity_status_,
    getOtherEntitiesCanonicalizedEntityStatuses());
  if (distance_to_stop_target_) {
    target_linear_speed = calculateTargetSpeed(canonicalized_entity_status_->getTwist().linear.x);
  } else {
    return BT::NodeStatus::FAILURE;
  }
  if (distance_to_front_entity) {
    if (distance_to_front_entity.value() <= distance_to_stop_target_.value()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  if (
    const auto distance_to_stopline =
      traffic_simulator::distance::distanceToStopLine(route_lanelets_, *trajectory)) {
    if (distance_to_stopline.value() <= distance_to_stop_target_.value()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  if (target_speed_) {
    if (target_speed_.value() > target_linear_speed.value()) {
      target_speed_ = target_linear_speed.value();
    }
  } else {
    target_speed_ = target_linear_speed.value();
  }
  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed_.value()));
  setOutput("waypoints", waypoints);
  setOutput("obstacle", calculateObstacle(waypoints));
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
