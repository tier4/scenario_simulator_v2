// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/yield_action.hpp>
#include <boost/algorithm/clamp.hpp>
#include <memory>
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

const boost::optional<traffic_simulator_msgs::msg::Obstacle> YieldAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
{
  if (!distance_to_stop_target_) {
    return boost::none;
  }
  if (distance_to_stop_target_.get() < 0) {
    return boost::none;
  }
  if (distance_to_stop_target_.get() > trajectory->getLength()) {
    return boost::none;
  }
  traffic_simulator_msgs::msg::Obstacle obstacle;
  obstacle.type = obstacle.ENTITY;
  obstacle.s = distance_to_stop_target_.get();
  return obstacle;
}

const traffic_simulator_msgs::msg::WaypointsArray YieldAction::calculateWaypoints()
{
  if (!entity_status.lanelet_pose_valid) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (entity_status.action_status.twist.linear.x >= 0) {
    traffic_simulator_msgs::msg::WaypointsArray waypoints;
    double horizon =
      boost::algorithm::clamp(entity_status.action_status.twist.linear.x * 5, 20, 50);
    waypoints.waypoints = reference_trajectory->getTrajectory(
      entity_status.lanelet_pose.s, entity_status.lanelet_pose.s + horizon, 1.0,
      entity_status.lanelet_pose.offset);
    trajectory = std::make_unique<traffic_simulator::math::CatmullRomSubspline>(
      reference_trajectory, entity_status.lanelet_pose.s, entity_status.lanelet_pose.s + horizon);
    return waypoints;
  } else {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
}

boost::optional<double> YieldAction::calculateTargetSpeed()
{
  if (!distance_to_stop_target_) {
    return boost::none;
  }
  double rest_distance =
    distance_to_stop_target_.get() - (vehicle_parameters.bounding_box.dimensions.x) - 10;
  if (rest_distance < calculateStopDistance(driver_model.deceleration)) {
    if (rest_distance > 0) {
      return std::sqrt(2 * driver_model.deceleration * rest_distance);
    } else {
      return 0;
    }
  }
  return entity_status.action_status.twist.linear.x;
}

BT::NodeStatus YieldAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  if (!driver_model.see_around) {
    return BT::NodeStatus::FAILURE;
  }
  if (!entity_status.lanelet_pose_valid) {
    return BT::NodeStatus::FAILURE;
  }
  const auto right_of_way_entities = getRightOfWayEntities(route_lanelets);
  if (right_of_way_entities.empty()) {
    if (!target_speed) {
      target_speed = hdmap_utils->getSpeedLimit(route_lanelets);
    }
    setOutput("updated_status", calculateEntityStatusUpdated(target_speed.get()));
    const auto waypoints = calculateWaypoints();
    if (waypoints.waypoints.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    const auto obstacle = calculateObstacle(waypoints);
    setOutput("waypoints", waypoints);
    setOutput("obstacle", obstacle);
    return BT::NodeStatus::SUCCESS;
  }
  distance_to_stop_target_ = getYieldStopDistance(route_lanelets);
  target_speed = calculateTargetSpeed();
  if (!target_speed) {
    target_speed = hdmap_utils->getSpeedLimit(route_lanelets);
  }
  setOutput("updated_status", calculateEntityStatusUpdated(target_speed.get()));
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
