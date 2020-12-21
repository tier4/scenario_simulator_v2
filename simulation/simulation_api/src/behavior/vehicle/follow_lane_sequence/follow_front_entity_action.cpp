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
#include <simulation_api/behavior/vehicle/follow_lane_sequence/follow_front_entity_action.hpp>
#include <simulation_api/math/catmull_rom_spline.hpp>

#include <boost/algorithm/clamp.hpp>

#include <string>

namespace entity_behavior
{
namespace vehicle
{
namespace follow_lane_sequence
{
FollowFrontEntityAction::FollowFrontEntityAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config) {}

const openscenario_msgs::msg::CatmullRomSpline FollowFrontEntityAction::calculateTrajectory() const
{
  if (!entity_status.lanelet_pose_valid) {
    throw BehaviorTreeRuntimeError("failed to assign lane");
  }
  double horizon = 0;
  if (entity_status.action_status.twist.linear.x > 0) {
    horizon = boost::algorithm::clamp(entity_status.action_status.twist.linear.x * 5, 0, 50);
    auto following_lanelets = hdmap_utils->getFollowingLanelets(
      entity_status.lanelet_pose.lanelet_id,
      horizon + hdmap_utils->getLaneletLength(entity_status.lanelet_pose.lanelet_id));
    simulation_api::math::CatmullRomSpline spline(hdmap_utils->getCenterPoints(following_lanelets));
    auto traj = spline.getTrajectory(entity_status.lanelet_pose.s,
        entity_status.lanelet_pose.s + horizon, 1.0);
    return simulation_api::math::CatmullRomSpline(traj).toRosMsg();
  } else {
    horizon = boost::algorithm::clamp(entity_status.action_status.twist.linear.x * 5, -5, 0);
    simulation_api::math::CatmullRomSpline spline(hdmap_utils->getCenterPoints(
        entity_status.lanelet_pose.lanelet_id));
    auto traj = spline.getTrajectory(entity_status.lanelet_pose.s,
        entity_status.lanelet_pose.s - horizon, 1.0);
    return simulation_api::math::CatmullRomSpline(traj).toRosMsg();
  }
}

BT::NodeStatus FollowFrontEntityAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  auto following_lanelets = hdmap_utils->getFollowingLanelets(entity_status.lanelet_pose.lanelet_id,
      50);
  if (getRightOfWayEntities(following_lanelets).size() != 0) {
    return BT::NodeStatus::FAILURE;
  }
  auto distance_to_stopline = hdmap_utils->getDistanceToStopLine(following_lanelets,
      entity_status.lanelet_pose.lanelet_id,
      entity_status.lanelet_pose.s);
  auto distance_to_crossing_entity = getDistanceToConflictingEntity(following_lanelets);
  auto distance_to_front_entity = getDistanceToFrontEntity();
  if (!distance_to_front_entity) {
    return BT::NodeStatus::FAILURE;
  }
  if (distance_to_crossing_entity) {
    if (distance_to_front_entity.get() > distance_to_crossing_entity.get()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  if (distance_to_stopline) {
    if (distance_to_front_entity.get() > distance_to_stopline.get()) {
      return BT::NodeStatus::FAILURE;
    }
  }
  auto front_entity_status = getFrontEntityStatus();
  if (!front_entity_status) {
    return BT::NodeStatus::FAILURE;
  }
  if (distance_to_front_entity.get() >=
    (calculateStopDistance() +
    vehicle_parameters->bounding_box.dimensions.length + 5))
  {
    auto entity_status_updated = calculateEntityStatusUpdated(
      front_entity_status.get().action_status.twist.linear.x + 2);
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  } else if (distance_to_front_entity.get() <= calculateStopDistance()) {
    auto entity_status_updated = calculateEntityStatusUpdated(
      front_entity_status.get().action_status.twist.linear.x - 2);
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  } else {
    auto entity_status_updated = calculateEntityStatusUpdated(
      front_entity_status.get().action_status.twist.linear.x);
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  }
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
