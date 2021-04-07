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

#include <boost/algorithm/clamp.hpp>
#include <string>
#include <traffic_simulator/behavior/vehicle/behavior_tree.hpp>
#include <traffic_simulator/behavior/vehicle/follow_lane_sequence/follow_front_entity_action.hpp>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
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

const boost::optional<openscenario_msgs::msg::Obstacle> FollowFrontEntityAction::calculateObstacle(
  const openscenario_msgs::msg::WaypointsArray & waypoints)
{
  if (!distance_to_front_entity_) {
    return boost::none;
  }
  if (distance_to_front_entity_.get() < 0) {
    return boost::none;
  }
  traffic_simulator::math::CatmullRomSpline spline(waypoints.waypoints);
  if (distance_to_front_entity_.get() > spline.getLength()) {
    return boost::none;
  }
  openscenario_msgs::msg::Obstacle obstacle;
  obstacle.type = obstacle.ENTITY;
  obstacle.s = distance_to_front_entity_.get();
  return obstacle;
}

const openscenario_msgs::msg::WaypointsArray FollowFrontEntityAction::calculateWaypoints()
{
  if (!entity_status.lanelet_pose_valid) {
    throw BehaviorTreeRuntimeError("failed to assign lane");
  }
  if (entity_status.action_status.twist.linear.x >= 0) {
    openscenario_msgs::msg::WaypointsArray waypoints;
    double horizon =
      boost::algorithm::clamp(entity_status.action_status.twist.linear.x * 5, 20, 50);
    traffic_simulator::math::CatmullRomSpline spline(hdmap_utils->getCenterPoints(route_lanelets));
    waypoints.waypoints = spline.getTrajectory(
      entity_status.lanelet_pose.s, entity_status.lanelet_pose.s + horizon, 1.0);
    return waypoints;
  } else {
    return openscenario_msgs::msg::WaypointsArray();
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
  auto distance_to_stopline =
    hdmap_utils->getDistanceToStopLine(route_lanelets, waypoints.waypoints);
  auto distance_to_crossing_entity = getDistanceToConflictingEntity(route_lanelets);
  distance_to_front_entity_ = getDistanceToFrontEntity();
  if (!distance_to_front_entity_) {
    return BT::NodeStatus::FAILURE;
  }
  if (distance_to_crossing_entity) {
    if (distance_to_front_entity_.get() > distance_to_crossing_entity.get()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  if (distance_to_stopline) {
    if (distance_to_front_entity_.get() > distance_to_stopline.get()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  auto front_entity_status = getFrontEntityStatus();
  if (!front_entity_status) {
    return BT::NodeStatus::FAILURE;
  }
  if (
    distance_to_front_entity_.get() >=
    (calculateStopDistance() + vehicle_parameters.bounding_box.dimensions.x + 5)) {
    auto entity_status_updated =
      calculateEntityStatusUpdated(front_entity_status.get().action_status.twist.linear.x + 2);
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  } else if (distance_to_front_entity_.get() <= calculateStopDistance()) {
    auto entity_status_updated =
      calculateEntityStatusUpdated(front_entity_status.get().action_status.twist.linear.x - 2);
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  } else {
    auto entity_status_updated =
      calculateEntityStatusUpdated(front_entity_status.get().action_status.twist.linear.x);
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
