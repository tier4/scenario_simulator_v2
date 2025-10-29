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
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/follow_lane_action.hpp>
#include <get_parameter/get_parameter.hpp>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/helper/stop_watch.hpp>
#include <traffic_simulator/utils/route.hpp>
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
  use_trajectory_based_front_entity_detection_ =
    common::getParameter<bool>("use_trajectory_based_front_entity_detection", false);
  trajectory_based_detection_offset_ =
    common::getParameter<double>("trajectory_based_detection_offset", 0.0);
}

const std::optional<traffic_simulator_msgs::msg::Obstacle> FollowLaneAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
{
  return std::nullopt;
}

const traffic_simulator_msgs::msg::WaypointsArray FollowLaneAction::calculateWaypoints()
{
  if (!canonicalized_entity_status_->isInLanelet()) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (canonicalized_entity_status_->getTwist().linear.x >= 0) {
    traffic_simulator_msgs::msg::WaypointsArray waypoints;
    const auto lanelet_pose = canonicalized_entity_status_->getLaneletPose();
    waypoints.waypoints = reference_trajectory->getTrajectory(
      lanelet_pose.s, lanelet_pose.s + getHorizon(), waypoint_interval, lanelet_pose.offset);
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

bool FollowLaneAction::checkPreconditions()
{
  if (
    request_ != traffic_simulator::behavior::Request::NONE &&
    request_ != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return false;
  } else {
    return true;
  }
}

BT::NodeStatus FollowLaneAction::doAction()
{
  if (!canonicalized_entity_status_->isInLanelet()) {
    stopEntity();
    const auto waypoints = traffic_simulator_msgs::msg::WaypointsArray{};
    setOutput("waypoints", waypoints);
    setOutput("obstacle", calculateObstacle(waypoints));
    return BT::NodeStatus::RUNNING;
  }
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (behavior_parameter_.see_around) {
    if (traffic_simulator::route::isNeedToRightOfWay(
          route_lanelets_, getOtherEntitiesCanonicalizedLaneletPoses())) {
      return BT::NodeStatus::FAILURE;
    }
    if (trajectory == nullptr) {
      return BT::NodeStatus::FAILURE;
    }
    std::optional<double> distance_to_front_entity;
    if (use_trajectory_based_front_entity_detection_) {
      constexpr std::size_t trajectory_segments = 50;
      if (
        const auto front_entity_info = getFrontEntityNameAndDistanceByTrajectory(
          waypoints.waypoints,
          vehicle_parameters.bounding_box.dimensions.y + trajectory_based_detection_offset_,
          trajectory_segments)) {
        distance_to_front_entity = front_entity_info->second;
      }
    } else {
      distance_to_front_entity = getDistanceToFrontEntity(*trajectory);
    }
    if (distance_to_front_entity) {
      if (
        distance_to_front_entity.value() <=
        calculateStopDistance(behavior_parameter_.dynamic_constraints) +
          vehicle_parameters.bounding_box.dimensions.x + front_entity_stop_margin) {
        return BT::NodeStatus::FAILURE;
      }
    }
    const auto distance_to_traffic_stop_line =
      traffic_lights_->getDistanceToActiveTrafficLightStopLine(route_lanelets_, *trajectory);
    if (distance_to_traffic_stop_line) {
      if (distance_to_traffic_stop_line.value() <= getHorizon()) {
        return BT::NodeStatus::FAILURE;
      }
    }

    if (
      const auto distance_to_stopline =
        traffic_simulator::distance::distanceToStopLine(route_lanelets_, *trajectory)) {
      if (
        distance_to_stopline.value() <=
        calculateStopDistance(behavior_parameter_.dynamic_constraints) +
          vehicle_parameters.bounding_box.dimensions.x * bounding_box_half_factor +
          stop_line_margin) {
        return BT::NodeStatus::FAILURE;
      }
    }
    if (
      const auto distance_to_conflicting_entity =
        traffic_simulator::distance::distanceToNearestConflictingPose(
          route_lanelets_, *trajectory, *canonicalized_entity_status_,
          getOtherEntitiesCanonicalizedEntityStatuses())) {
      if (
        distance_to_conflicting_entity.value() <
        (vehicle_parameters.bounding_box.dimensions.x + conflicting_entity_margin +
         calculateStopDistance(behavior_parameter_.dynamic_constraints))) {
        return BT::NodeStatus::FAILURE;
      }
    }
  }
  if (!target_speed_) {
    target_speed_ = traffic_simulator::route::speedLimit(route_lanelets_);
  }
  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed_.value()));
  setOutput("waypoints", waypoints);
  setOutput("obstacle", calculateObstacle(waypoints));
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
