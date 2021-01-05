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

#include <simulation_api/behavior/vehicle/behavior_tree.hpp>
#include <simulation_api/behavior/vehicle/acquire_position_action.hpp>
#include <simulation_api/math/catmull_rom_spline.hpp>

#include <openscenario_msgs/msg/waypoints_array.hpp>

#include <boost/algorithm/clamp.hpp>

#include <string>
#include <vector>
#include <memory>

namespace entity_behavior
{
namespace vehicle
{
AcquirePositionAction::AcquirePositionAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config) {}

const openscenario_msgs::msg::WaypointsArray AcquirePositionAction::calculateWaypoints()
{
  if (!entity_status.lanelet_pose_valid) {
    throw BehaviorTreeRuntimeError("failed to assign lane");
  }
  if (entity_status.action_status.twist.linear.x >= 0) {
    openscenario_msgs::msg::WaypointsArray waypoints;
    double horizon =
      boost::algorithm::clamp(entity_status.action_status.twist.linear.x * 5, 20, 50);
    auto following_lanelets = hdmap_utils->getFollowingLanelets(
      entity_status.lanelet_pose.lanelet_id, route_,
      horizon + hdmap_utils->getLaneletLength(entity_status.lanelet_pose.lanelet_id));
    simulation_api::math::CatmullRomSpline spline(hdmap_utils->getCenterPoints(following_lanelets));
    waypoints.waypoints = spline.getTrajectory(
      entity_status.lanelet_pose.s,
      entity_status.lanelet_pose.s + horizon, 1.0);
    return waypoints;
  } else {
    return openscenario_msgs::msg::WaypointsArray();
  }
}

void AcquirePositionAction::getBlackBoardValues()
{
  openscenario_msgs::msg::LaneletPose target_lanelet_pose;
  VehicleActionNode::getBlackBoardValues();
  if (!getInput<openscenario_msgs::msg::LaneletPose>("target_lanelet_pose", target_lanelet_pose)) {
    target_lanelet_pose_ = boost::none;
  } else {
    target_lanelet_pose_ = target_lanelet_pose;
  }
}

BT::NodeStatus AcquirePositionAction::tick()
{
  getBlackBoardValues();
  if (request != "acquire_position") {
    target_lanelet_pose_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }
  route_ = hdmap_utils->getRoute(
    entity_status.lanelet_pose.lanelet_id,
    target_lanelet_pose_->lanelet_id);
  std::vector<std::int64_t> following_lanelets;
  following_lanelets = hdmap_utils->getFollowingLanelets(
    entity_status.lanelet_pose.lanelet_id, route_);
  if (!target_speed) {
    target_speed = hdmap_utils->getSpeedLimit(following_lanelets);
  }
  auto distance_to_front_entity = getDistanceToFrontEntity();
  if (distance_to_front_entity) {
    if (distance_to_front_entity.get() <=
      calculateStopDistance() +
      vehicle_parameters->bounding_box.dimensions.length + 5)
    {
      auto front_entity_status = getFrontEntityStatus();
      if (front_entity_status) {
        target_speed = front_entity_status->action_status.twist.linear.x;
      }
    }
  }
  auto distance_to_conflicting_entity = getDistanceToConflictingEntity(following_lanelets);
  if (distance_to_conflicting_entity) {
    if (distance_to_conflicting_entity.get() <=
      calculateStopDistance() +
      vehicle_parameters->bounding_box.dimensions.length + 5)
    {
      target_speed = 0;
    }
  }
  if (getRightOfWayEntities(following_lanelets).size() != 0) {
    return BT::NodeStatus::FAILURE;
  }
  auto entity_status_updated = calculateEntityStatusUpdated(target_speed.get(), route_);
  setOutput("updated_status", entity_status_updated);
  const auto waypoints = calculateWaypoints();
  const auto obstacles = calculateObstacles(waypoints);
  setOutput("waypoints", waypoints);
  setOutput("obstacles", obstacles);
  if (target_lanelet_pose_->lanelet_id == entity_status.lanelet_pose.lanelet_id) {
    if (target_lanelet_pose_->s < entity_status.lanelet_pose.s) {
      target_lanelet_pose_ = boost::none;
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::RUNNING;
}
}      // namespace vehicle
}  // namespace entity_behavior
