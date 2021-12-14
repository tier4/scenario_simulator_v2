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
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/stop_at_stop_line_action.hpp>
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
StopAtStopLineAction::StopAtStopLineAction(
  const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config)
{
  stopped_ = false;
}

const boost::optional<traffic_simulator_msgs::msg::Obstacle>
StopAtStopLineAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints)
{
  if (!distance_to_stopline_) {
    return boost::none;
  }
  if (distance_to_stopline_.get() < 0) {
    return boost::none;
  }
  traffic_simulator::math::CatmullRomSpline spline(waypoints.waypoints);
  if (distance_to_stopline_.get() > spline.getLength()) {
    return boost::none;
  }
  traffic_simulator_msgs::msg::Obstacle obstacle;
  obstacle.type = obstacle.ENTITY;
  obstacle.s = distance_to_stopline_.get();
  return obstacle;
}

const traffic_simulator_msgs::msg::WaypointsArray StopAtStopLineAction::calculateWaypoints()
{
  if (!entity_status.lanelet_pose_valid) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (entity_status.action_status.twist.linear.x >= 0) {
    traffic_simulator_msgs::msg::WaypointsArray waypoints;
    double horizon =
      boost::algorithm::clamp(entity_status.action_status.twist.linear.x * 5, 20, 50);
    traffic_simulator::math::CatmullRomSpline spline(hdmap_utils->getCenterPoints(route_lanelets));
    waypoints.waypoints = spline.getTrajectory(
      entity_status.lanelet_pose.s, entity_status.lanelet_pose.s + horizon, 1.0);
    return waypoints;
  } else {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
}

boost::optional<double> StopAtStopLineAction::calculateTargetSpeed(double current_velocity)
{
  if (!distance_to_stopline_) {
    return boost::none;
  }
  double rest_distance =
    distance_to_stopline_.get() - (vehicle_parameters.bounding_box.dimensions.x);
  if (rest_distance < calculateStopDistance()) {
    if (rest_distance > 0) {
      return std::sqrt(2 * driver_model.deceleration * rest_distance);
    } else {
      return 0;
    }
  }
  return current_velocity;
}

BT::NodeStatus StopAtStopLineAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    stopped_ = false;
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
  distance_to_stopline_ = hdmap_utils->getDistanceToStopLine(route_lanelets, waypoints.waypoints);
  if (std::fabs(entity_status.action_status.twist.linear.x) < 0.001) {
    if (distance_to_stopline_) {
      if (distance_to_stopline_.get() <= vehicle_parameters.bounding_box.dimensions.x + 5) {
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
      setOutput("updated_status", calculateEntityStatusUpdated(target_speed.get()));
      const auto obstacle = calculateObstacle(waypoints);
      setOutput("waypoints", waypoints);
      setOutput("obstacle", obstacle);
      return BT::NodeStatus::SUCCESS;
    }
    setOutput("updated_status", calculateEntityStatusUpdated(target_speed.get()));
    const auto obstacle = calculateObstacle(waypoints);
    setOutput("waypoints", waypoints);
    setOutput("obstacle", obstacle);
    return BT::NodeStatus::RUNNING;
  }
  auto target_linear_speed = calculateTargetSpeed(entity_status.action_status.twist.linear.x);
  if (!target_linear_speed) {
    stopped_ = false;
    return BT::NodeStatus::FAILURE;
  }
  if (target_speed) {
    if (target_speed.get() > target_linear_speed.get()) {
      target_speed = target_linear_speed.get();
    }
  } else {
    target_speed = target_linear_speed.get();
  }
  setOutput("updated_status", calculateEntityStatusUpdated(target_speed.get()));
  stopped_ = false;
  const auto obstacle = calculateObstacle(waypoints);
  setOutput("waypoints", waypoints);
  setOutput("obstacle", obstacle);
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
