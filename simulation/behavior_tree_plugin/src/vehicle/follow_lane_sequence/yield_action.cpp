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
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/yield_action.hpp>
#include <memory>
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
YieldAction::YieldAction(const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config)
{
}

const std::optional<traffic_simulator_msgs::msg::Obstacle> YieldAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
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

const traffic_simulator_msgs::msg::WaypointsArray YieldAction::calculateWaypoints()
{
  if (!canonicalized_entity_status_->isInLanelet()) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (canonicalized_entity_status_->getTwist().linear.x >= 0) {
    traffic_simulator_msgs::msg::WaypointsArray waypoints;
    double horizon = getHorizon();
    const auto lanelet_pose = canonicalized_entity_status_->getLaneletPose();
    waypoints.waypoints = reference_trajectory->getTrajectory(
      lanelet_pose.s, lanelet_pose.s + horizon, 1.0, lanelet_pose.offset);
    trajectory = std::make_unique<math::geometry::CatmullRomSubspline>(
      reference_trajectory, lanelet_pose.s, lanelet_pose.s + horizon);
    return waypoints;
  } else {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
}

std::optional<double> YieldAction::calculateTargetSpeed()
{
  if (!distance_to_stop_target_) {
    return std::nullopt;
  }
  /**
   * @brief hard coded parameter!! 1.0 is a stop margin
   */
  double rest_distance =
    distance_to_stop_target_.value() - (vehicle_parameters.bounding_box.dimensions.x * 0.5 + 1.0);
  if (rest_distance < calculateStopDistance(behavior_parameter_.dynamic_constraints)) {
    return 0;
  }
  return canonicalized_entity_status_->getTwist().linear.x;
}

bool YieldAction::checkPreconditions()
{
  if (
    request_ != traffic_simulator::behavior::Request::NONE &&
    request_ != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return false;
  } else if (!behavior_parameter_.see_around) {
    return false;
  } else if (!canonicalized_entity_status_->isInLanelet()) {
    return false;
  } else {
    return true;
  }
}

BT::NodeStatus YieldAction::doAction()
{
  const auto right_of_way_entities = getRightOfWayEntities(route_lanelets_);
  if (right_of_way_entities.empty()) {
    if (!target_speed_) {
      target_speed_ = hdmap_utils_->getSpeedLimit(route_lanelets_);
    }
    setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed_.value()));
    const auto waypoints = calculateWaypoints();
    if (waypoints.waypoints.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    const auto obstacle = calculateObstacle(waypoints);
    setOutput("waypoints", waypoints);
    setOutput("obstacle", obstacle);
    return BT::NodeStatus::SUCCESS;
  }
  distance_to_stop_target_ = getYieldStopDistance(route_lanelets_);
  target_speed_ = calculateTargetSpeed();
  if (!target_speed_) {
    target_speed_ = hdmap_utils_->getSpeedLimit(route_lanelets_);
  }
  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed_.value()));
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  const auto obstacle = calculateObstacle(waypoints);
  setOutput("waypoints", waypoints);
  setOutput("obstacle", obstacle);
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
