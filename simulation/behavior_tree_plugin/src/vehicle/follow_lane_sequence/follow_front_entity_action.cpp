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
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/follow_front_entity_action.hpp>
#include <boost/algorithm/clamp.hpp>
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

const boost::optional<traffic_simulator_msgs::msg::Obstacle>
FollowFrontEntityAction::calculateObstacle(const traffic_simulator_msgs::msg::WaypointsArray &)
{
  if (!distance_to_front_entity_) {
    return boost::none;
  }
  if (distance_to_front_entity_.get() < 0) {
    return boost::none;
  }
  if (distance_to_front_entity_.get() > trajectory->getLength()) {
    return boost::none;
  }
  traffic_simulator_msgs::msg::Obstacle obstacle;
  obstacle.type = obstacle.ENTITY;
  obstacle.s = distance_to_front_entity_.get();
  return obstacle;
}

const traffic_simulator_msgs::msg::WaypointsArray FollowFrontEntityAction::calculateWaypoints()
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

BT::NodeStatus FollowFrontEntityAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  if (getRightOfWayEntities(route_lanelets).size() != 0) {
    return BT::NodeStatus::FAILURE;
  }
  if (!driver_model.see_around) {
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
    getDistanceToTargetEntityPolygon(*trajectory, front_entity_name.get());
  if (!distance_to_front_entity_) {
    return BT::NodeStatus::FAILURE;
  }
  if (distance_to_conflicting_entity) {
    if (distance_to_front_entity_.get() > distance_to_conflicting_entity.get()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  if (distance_to_stopline) {
    if (distance_to_front_entity_.get() > distance_to_stopline.get()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  auto front_entity_status = getEntityStatus(front_entity_name.get());
  if (!target_speed) {
    target_speed = hdmap_utils->getSpeedLimit(route_lanelets);
  }
  if (target_speed.get() <= front_entity_status.action_status.twist.linear.x) {
    auto entity_status_updated = calculateEntityStatusUpdated(target_speed.get());
    setOutput("updated_status", entity_status_updated);
    const auto obstacle = calculateObstacle(waypoints);
    setOutput("waypoints", waypoints);
    setOutput("obstacle", obstacle);
    return BT::NodeStatus::RUNNING;
  }
  if (
    distance_to_front_entity_.get() >= (calculateStopDistance(driver_model.deceleration) +
                                        vehicle_parameters.bounding_box.dimensions.x + 5)) {
    auto entity_status_updated =
      calculateEntityStatusUpdated(front_entity_status.action_status.twist.linear.x + 2);
    setOutput("updated_status", entity_status_updated);
    const auto obstacle = calculateObstacle(waypoints);
    setOutput("waypoints", waypoints);
    setOutput("obstacle", obstacle);
    return BT::NodeStatus::RUNNING;
  } else if (distance_to_front_entity_.get() <= calculateStopDistance(driver_model.deceleration)) {
    auto entity_status_updated =
      calculateEntityStatusUpdated(front_entity_status.action_status.twist.linear.x - 2);
    setOutput("updated_status", entity_status_updated);
    const auto obstacle = calculateObstacle(waypoints);
    setOutput("waypoints", waypoints);
    setOutput("obstacle", obstacle);
    return BT::NodeStatus::RUNNING;
  } else {
    auto entity_status_updated =
      calculateEntityStatusUpdated(front_entity_status.action_status.twist.linear.x);
    setOutput("updated_status", entity_status_updated);
    const auto obstacle = calculateObstacle(waypoints);
    setOutput("waypoints", waypoints);
    setOutput("obstacle", obstacle);
    return BT::NodeStatus::RUNNING;
  }
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
