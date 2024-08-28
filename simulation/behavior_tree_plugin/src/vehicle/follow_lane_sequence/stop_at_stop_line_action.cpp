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
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/stop_at_stop_line_action.hpp>
#include <optional>
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
StopAtStopLineAction::StopAtStopLineAction(
  const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config)
{
  stopped_ = false;
}

const std::optional<traffic_simulator_msgs::msg::Obstacle> StopAtStopLineAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
{
  if (!distance_to_stopline_) {
    return std::nullopt;
  }
  if (distance_to_stopline_.value() < 0) {
    return std::nullopt;
  }
  if (distance_to_stopline_.value() > trajectory->getLength()) {
    return std::nullopt;
  }
  traffic_simulator_msgs::msg::Obstacle obstacle;
  obstacle.type = obstacle.ENTITY;
  obstacle.s = distance_to_stopline_.value();
  return obstacle;
}

const traffic_simulator_msgs::msg::WaypointsArray StopAtStopLineAction::calculateWaypoints()
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

std::optional<double> StopAtStopLineAction::calculateTargetSpeed(double current_velocity)
{
  if (!distance_to_stopline_) {
    return std::nullopt;
  }
  /**
   * @brief hard coded parameter!! 1.0 is a stop margin
   */
  double rest_distance =
    distance_to_stopline_.value() - (vehicle_parameters.bounding_box.dimensions.x * 0.5 + 1.0);
  if (rest_distance < calculateStopDistance(behavior_parameter.dynamic_constraints)) {
    return 0;
  }
  return current_velocity;
}

BT::NodeStatus StopAtStopLineAction::tick()
{
  getBlackBoardValues();
  if (
    request != traffic_simulator::behavior::Request::NONE &&
    request != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    stopped_ = false;
    return BT::NodeStatus::FAILURE;
  }
  if (!behavior_parameter.see_around) {
    return BT::NodeStatus::FAILURE;
  }
  if (getRightOfWayEntities(route_lanelets).size() != 0) {
    return BT::NodeStatus::FAILURE;
  }
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (trajectory == nullptr) {
    return BT::NodeStatus::FAILURE;
  }
  distance_to_stopline_ = hdmap_utils->getDistanceToStopLine(route_lanelets, *trajectory);
  const auto distance_to_stop_target = getDistanceToConflictingEntity(route_lanelets, *trajectory);
  const auto distance_to_front_entity = getDistanceToFrontEntity(*trajectory);
  if (!distance_to_stopline_) {
    stopped_ = false;
    return BT::NodeStatus::FAILURE;
  }
  if (distance_to_front_entity) {
    if (distance_to_front_entity.value() <= distance_to_stopline_.value()) {
      stopped_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }
  if (distance_to_stop_target) {
    if (distance_to_stop_target.value() <= distance_to_stopline_.value()) {
      stopped_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }

  if (std::fabs(canonicalized_entity_status->getTwist().linear.x) < 0.001) {
    if (distance_to_stopline_) {
      if (distance_to_stopline_.value() <= vehicle_parameters.bounding_box.dimensions.x + 5) {
        stopped_ = true;
      }
    }
  }
  if (stopped_) {
    if (!target_speed) {
      target_speed = hdmap_utils->getSpeedLimit(route_lanelets);
    }
    if (!distance_to_stopline_) {
      stopped_ = false;
      setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed.value()));
      setOutput("waypoints", waypoints);
      setOutput("obstacle", calculateObstacle(waypoints));
      return BT::NodeStatus::SUCCESS;
    }
    setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed.value()));
    setOutput("waypoints", waypoints);
    setOutput("obstacle", calculateObstacle(waypoints));
    return BT::NodeStatus::RUNNING;
  }
  auto target_linear_speed = calculateTargetSpeed(canonicalized_entity_status->getTwist().linear.x);
  if (!target_linear_speed) {
    stopped_ = false;
    return BT::NodeStatus::FAILURE;
  }
  if (target_speed) {
    if (target_speed.value() > target_linear_speed.value()) {
      target_speed = target_linear_speed.value();
    }
  } else {
    target_speed = target_linear_speed.value();
  }
  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed.value()));
  stopped_ = false;
  setOutput("waypoints", waypoints);
  setOutput("obstacle", calculateObstacle(waypoints));
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
