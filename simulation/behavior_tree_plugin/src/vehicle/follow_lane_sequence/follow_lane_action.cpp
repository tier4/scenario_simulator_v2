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
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/follow_lane_action.hpp>
#include <optional>
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

const std::optional<traffic_simulator_msgs::msg::Obstacle> FollowLaneAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
{
  return std::nullopt;
}

const traffic_simulator_msgs::msg::WaypointsArray FollowLaneAction::calculateWaypoints()
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

void FollowLaneAction::getBlackBoardValues()
{
  traffic_simulator::LaneletPose target_lanelet_pose;
  VehicleActionNode::getBlackBoardValues();
  if (!getInput<traffic_simulator::LaneletPose>("target_lanelet_pose", target_lanelet_pose)) {
    target_lanelet_pose_ = std::nullopt;
  } else {
    target_lanelet_pose_ = target_lanelet_pose;
  }
}

BT::NodeStatus FollowLaneAction::tick()
{
  getBlackBoardValues();
  if (
    request != traffic_simulator::behavior::Request::NONE &&
    request != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return BT::NodeStatus::FAILURE;
  }
  if (!canonicalized_entity_status->laneMatchingSucceed()) {
    stopEntity();
    return BT::NodeStatus::RUNNING;
  }
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (behavior_parameter.see_around) {
    if (getRightOfWayEntities(route_lanelets).size() != 0) {
      return BT::NodeStatus::FAILURE;
    }
    if (trajectory == nullptr) {
      return BT::NodeStatus::FAILURE;
    }
    auto distance_to_front_entity = getDistanceToFrontEntity(*trajectory);
    if (distance_to_front_entity) {
      if (
        distance_to_front_entity.value() <=
        calculateStopDistance(behavior_parameter.dynamic_constraints) +
          vehicle_parameters.bounding_box.dimensions.x + 5) {
        return BT::NodeStatus::FAILURE;
      }
    }
    const auto distance_to_traffic_stop_line =
      getDistanceToTrafficLightStopLine(route_lanelets, *trajectory);
    if (distance_to_traffic_stop_line) {
      if (distance_to_traffic_stop_line.value() <= getHorizon()) {
        return BT::NodeStatus::FAILURE;
      }
    }
    auto distance_to_stopline = hdmap_utils->getDistanceToStopLine(route_lanelets, *trajectory);
    auto distance_to_conflicting_entity =
      getDistanceToConflictingEntity(route_lanelets, *trajectory);
    if (distance_to_stopline) {
      if (
        distance_to_stopline.value() <=
        calculateStopDistance(behavior_parameter.dynamic_constraints) +
          vehicle_parameters.bounding_box.dimensions.x * 0.5 + 5) {
        return BT::NodeStatus::FAILURE;
      }
    }
    if (distance_to_conflicting_entity) {
      if (
        distance_to_conflicting_entity.value() <
        (vehicle_parameters.bounding_box.dimensions.x + 3 +
         calculateStopDistance(behavior_parameter.dynamic_constraints))) {
        return BT::NodeStatus::FAILURE;
      }
    }
  }
  if (!target_speed) {
    target_speed = hdmap_utils->getSpeedLimit(route_lanelets);
  }
  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed.value()));
  setOutput("waypoints", waypoints);
  setOutput("obstacle", calculateObstacle(waypoints));
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
