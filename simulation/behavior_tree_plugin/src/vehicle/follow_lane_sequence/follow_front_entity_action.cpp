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
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/follow_front_entity_action.hpp>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/route.hpp>
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

const std::optional<traffic_simulator_msgs::msg::Obstacle>
FollowFrontEntityAction::calculateObstacle(const traffic_simulator_msgs::msg::WaypointsArray &)
{
  if (!distance_to_front_entity_) {
    return std::nullopt;
  }
  if (distance_to_front_entity_.value() < 0) {
    return std::nullopt;
  }
  if (distance_to_front_entity_.value() > trajectory->getLength()) {
    return std::nullopt;
  }
  traffic_simulator_msgs::msg::Obstacle obstacle;
  obstacle.type = obstacle.ENTITY;
  obstacle.s = distance_to_front_entity_.value();
  return obstacle;
}

const traffic_simulator_msgs::msg::WaypointsArray FollowFrontEntityAction::calculateWaypoints()
{
  if (!canonicalized_entity_status_->isInLanelet()) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (canonicalized_entity_status_->getTwist().linear.x >= 0) {
    traffic_simulator_msgs::msg::WaypointsArray waypoints;
    double horizon = getHorizon();
    const auto lanelet_pose = canonicalized_entity_status_->getLaneletPose();
    waypoints.waypoints = reference_trajectory->getTrajectory(
      lanelet_pose.s, lanelet_pose.s + horizon, waypoint_interval, lanelet_pose.offset);
    trajectory = std::make_unique<math::geometry::CatmullRomSubspline>(
      reference_trajectory, lanelet_pose.s, lanelet_pose.s + horizon);
    return waypoints;
  } else {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
}

bool FollowFrontEntityAction::checkPreconditions()
{
  if (
    request_ != traffic_simulator::behavior::Request::NONE &&
    request_ != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return false;
  } else if (traffic_simulator::route::isNeedToRightOfWay(
               route_lanelets_, getOtherEntitiesCanonicalizedLaneletPoses())) {
    return false;
  } else if (!behavior_parameter_.see_around) {
    return false;
  } else {
    return true;
  }
}

BT::NodeStatus FollowFrontEntityAction::doAction()
{
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (trajectory == nullptr) {
    return BT::NodeStatus::FAILURE;
  }
  const auto distance_to_stopline =
    traffic_simulator::distance::distanceToStopLine(route_lanelets_, *trajectory);
  const auto distance_to_conflicting_entity =
    traffic_simulator::distance::distanceToNearestConflictingPose(
      route_lanelets_, *trajectory, *canonicalized_entity_status_,
      getOtherEntitiesCanonicalizedEntityStatuses());
  const auto front_entity_name = getFrontEntityName(*trajectory);
  if (!front_entity_name) {
    return BT::NodeStatus::FAILURE;
  }
  const auto & front_entity_status = getEntityStatus(front_entity_name.value());
  distance_to_front_entity_ = traffic_simulator::distance::splineDistanceToBoundingBox(
    *trajectory, canonicalized_entity_status_->getCanonicalizedLaneletPose().value(),
    canonicalized_entity_status_->getBoundingBox(),
    front_entity_status.getCanonicalizedLaneletPose().value(),
    front_entity_status.getBoundingBox());
  if (!distance_to_front_entity_) {
    return BT::NodeStatus::FAILURE;
  }
  if (distance_to_conflicting_entity) {
    if (distance_to_front_entity_.value() > distance_to_conflicting_entity.value()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  if (distance_to_stopline) {
    if (distance_to_front_entity_.value() > distance_to_stopline.value()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  if (!target_speed_) {
    target_speed_ = traffic_simulator::route::speedLimit(route_lanelets_);
  }
  const double front_entity_linear_velocity = front_entity_status.getTwist().linear.x;
  if (target_speed_.value() <= front_entity_linear_velocity) {
    setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed_.value()));
    const auto obstacle = calculateObstacle(waypoints);
    setOutput("waypoints", waypoints);
    setOutput("obstacle", obstacle);
    return BT::NodeStatus::RUNNING;
  }
  if (
    distance_to_front_entity_.value() >=
    (calculateStopDistance(behavior_parameter_.dynamic_constraints) +
     vehicle_parameters.bounding_box.dimensions.x + front_entity_margin)) {
    setCanonicalizedEntityStatus(
      calculateUpdatedEntityStatus(front_entity_linear_velocity + speed_step));
    setOutput("waypoints", waypoints);
    setOutput("obstacle", calculateObstacle(waypoints));
    return BT::NodeStatus::RUNNING;
  } else if (
    distance_to_front_entity_.value() <=
    calculateStopDistance(behavior_parameter_.dynamic_constraints)) {
    setCanonicalizedEntityStatus(
      calculateUpdatedEntityStatus(front_entity_linear_velocity - speed_step));
    setOutput("waypoints", waypoints);
    setOutput("obstacle", calculateObstacle(waypoints));
    return BT::NodeStatus::RUNNING;
  } else {
    setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(front_entity_linear_velocity));
    setOutput("waypoints", waypoints);
    setOutput("obstacle", calculateObstacle(waypoints));
    return BT::NodeStatus::RUNNING;
  }
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
