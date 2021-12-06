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
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/stop_at_traffic_light_action.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
#include <utility>
#include <vector>

namespace entity_behavior
{
namespace vehicle
{
namespace follow_lane_sequence
{
StopAtTrafficLightAction::StopAtTrafficLightAction(
  const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config)
{
}

const boost::optional<traffic_simulator_msgs::msg::Obstacle>
StopAtTrafficLightAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints)
{
  if (!distance_to_stop_target_) {
    return boost::none;
  }
  if (distance_to_stop_target_.get() < 0) {
    return boost::none;
  }
  traffic_simulator::math::CatmullRomSpline spline(waypoints.waypoints);
  if (distance_to_stop_target_.get() > spline.getLength()) {
    return boost::none;
  }
  traffic_simulator_msgs::msg::Obstacle obstacle;
  obstacle.type = obstacle.ENTITY;
  obstacle.s = distance_to_stop_target_.get();
  return obstacle;
}

const traffic_simulator_msgs::msg::WaypointsArray StopAtTrafficLightAction::calculateWaypoints()
{
  if (!entity_status.lanelet_pose_valid) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (entity_status.action_status.twist.linear.x >= 0) {
    traffic_simulator_msgs::msg::WaypointsArray waypoints;
    traffic_simulator::math::CatmullRomSpline spline(hdmap_utils->getCenterPoints(route_lanelets));
    waypoints.waypoints = spline.getTrajectory(
      entity_status.lanelet_pose.s, entity_status.lanelet_pose.s + getHorizon(), 1.0);
    return waypoints;
  } else {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
}

boost::optional<double> StopAtTrafficLightAction::calculateTargetSpeed(double current_velocity)
{
  if (!distance_to_stop_target_) {
    return boost::none;
  }
  double rest_distance =
    distance_to_stop_target_.get() - (vehicle_parameters.bounding_box.dimensions.x + 3);
  if (rest_distance < calculateStopDistance()) {
    if (rest_distance > 0) {
      return std::sqrt(2 * driver_model.deceleration * rest_distance);
    } else {
      return 0;
    }
  }
  return current_velocity;
}

BT::NodeStatus StopAtTrafficLightAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  if (!entity_status.lanelet_pose_valid) {
    return BT::NodeStatus::FAILURE;
  }
  if (!driver_model.see_around) {
    return BT::NodeStatus::FAILURE;
  }
  if (getRightOfWayEntities(route_lanelets).size() != 0) {
    return BT::NodeStatus::FAILURE;
  }
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  const auto spline = traffic_simulator::math::CatmullRomSpline(waypoints.waypoints);
  const auto distance_to_traffic_stop_line =
    hdmap_utils->getDistanceToTrafficLightStopLine(route_lanelets, waypoints.waypoints);
  if (!distance_to_traffic_stop_line) {
    return BT::NodeStatus::FAILURE;
  }
  distance_to_stop_target_ = getDistanceToTrafficLightStopLine(route_lanelets, waypoints.waypoints);
  boost::optional<double> target_linear_speed;
  if (distance_to_stop_target_) {
    if (distance_to_stop_target_.get() > getHorizon()) {
      return BT::NodeStatus::FAILURE;
    }
    target_linear_speed = calculateTargetSpeed(entity_status.action_status.twist.linear.x);
  } else {
    return BT::NodeStatus::FAILURE;
  }
  if (!distance_to_stop_target_) {
    setOutput("updated_status", calculateEntityStatusUpdated(0));
    const auto obstacle = calculateObstacle(waypoints);
    setOutput("waypoints", waypoints);
    setOutput("obstacle", obstacle);
    return BT::NodeStatus::SUCCESS;
  }
  if (target_speed) {
    if (target_speed.get() > target_linear_speed.get()) {
      target_speed = target_linear_speed.get();
    }
  } else {
    target_speed = target_linear_speed.get();
  }
  setOutput("updated_status", calculateEntityStatusUpdated(target_speed.get()));
  const auto obstacle = calculateObstacle(waypoints);
  setOutput("waypoints", waypoints);
  setOutput("obstacle", obstacle);
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
