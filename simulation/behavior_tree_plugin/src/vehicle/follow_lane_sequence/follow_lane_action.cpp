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
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/follow_lane_action.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/helper/stop_watch.hpp>
#include <vector>

namespace entity_behavior
{
namespace vehicle
{
namespace follow_lane_sequence
{
FollowLaneAction::FollowLaneAction(const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config)
{
}

const boost::optional<traffic_simulator_msgs::msg::Obstacle> FollowLaneAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
{
  return boost::none;
}

const traffic_simulator_msgs::msg::WaypointsArray FollowLaneAction::calculateWaypoints()
{
  if (!entity_status.lanelet_pose_valid) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (entity_status.action_status.twist.linear.x >= 0) {
    traffic_simulator_msgs::msg::WaypointsArray waypoints;
    waypoints.waypoints = reference_trajectory->getTrajectory(
      entity_status.lanelet_pose.s, entity_status.lanelet_pose.s + getHorizon(), 1.0,
      entity_status.lanelet_pose.offset);
    trajectory = std::make_unique<traffic_simulator::math::CatmullRomSubspline>(
      reference_trajectory, entity_status.lanelet_pose.s,
      entity_status.lanelet_pose.s + getHorizon());
    return waypoints;
  } else {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
}

void FollowLaneAction::getBlackBoardValues()
{
  traffic_simulator_msgs::msg::LaneletPose target_lanelet_pose;
  VehicleActionNode::getBlackBoardValues();
  if (!getInput<traffic_simulator_msgs::msg::LaneletPose>(
        "target_lanelet_pose", target_lanelet_pose)) {
    target_lanelet_pose_ = boost::none;
  } else {
    target_lanelet_pose_ = target_lanelet_pose;
  }
}

BT::NodeStatus FollowLaneAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  if (!entity_status.lanelet_pose_valid) {
    setOutput("updated_status", stopAtEndOfRoad());
    return BT::NodeStatus::RUNNING;
  }
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (driver_model.see_around) {
    if (getRightOfWayEntities(route_lanelets).size() != 0) {
      return BT::NodeStatus::FAILURE;
    }
    if (trajectory == nullptr) {
      return BT::NodeStatus::FAILURE;
    }
    auto distance_to_front_entity = getDistanceToFrontEntity(*trajectory);
    if (distance_to_front_entity) {
      if (
        distance_to_front_entity.get() <= calculateStopDistance(driver_model.deceleration) +
                                            vehicle_parameters.bounding_box.dimensions.x + 5) {
        return BT::NodeStatus::FAILURE;
      }
    }
    const auto distance_to_traffic_stop_line =
      getDistanceToTrafficLightStopLine(route_lanelets, *trajectory);
    if (distance_to_traffic_stop_line) {
      if (distance_to_traffic_stop_line.get() <= getHorizon()) {
        return BT::NodeStatus::FAILURE;
      }
    }
    auto distance_to_stopline = hdmap_utils->getDistanceToStopLine(route_lanelets, *trajectory);
    auto distance_to_conflicting_entity =
      getDistanceToConflictingEntity(route_lanelets, *trajectory);
    if (distance_to_stopline) {
      if (
        distance_to_stopline.get() <= calculateStopDistance(driver_model.deceleration) +
                                        vehicle_parameters.bounding_box.dimensions.x * 0.5 + 5) {
        return BT::NodeStatus::FAILURE;
      }
    }
    if (distance_to_conflicting_entity) {
      if (
        distance_to_conflicting_entity.get() < (vehicle_parameters.bounding_box.dimensions.x +
                                                calculateStopDistance(driver_model.deceleration))) {
        return BT::NodeStatus::FAILURE;
      }
    }
  }
  if (!target_speed) {
    target_speed = hdmap_utils->getSpeedLimit(route_lanelets);
  }
  auto updated_status = calculateEntityStatusUpdated(target_speed.get());
  setOutput("updated_status", updated_status);
  const auto obstacle = calculateObstacle(waypoints);
  setOutput("waypoints", waypoints);
  setOutput("obstacle", obstacle);
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
