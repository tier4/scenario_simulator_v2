// Copyright 2015-2020 TierIV.inc. All rights reserved.
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
#include <simulation_api/behavior/vehicle/follow_lane_sequence/follow_lane_action.hpp>

#include <string>


namespace entity_behavior
{
namespace vehicle
{
namespace follow_lane_sequence
{
FollowLaneAction::FollowLaneAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config) {}

BT::NodeStatus FollowLaneAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::WORLD) {
    if (target_speed) {
      if (target_speed.get() > vehicle_parameters->performance.max_speed) {
        target_speed = vehicle_parameters->performance.max_speed;
      }
    } else {
      target_speed = vehicle_parameters->performance.max_speed;
    }
    setOutput("updated_status", calculateEntityStatusUpdated(target_speed.get()));
    return BT::NodeStatus::RUNNING;
  }
  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::LANE) {
    auto following_lanelets = hdmap_utils->getFollowingLanelets(entity_status.lanelet_id, 50);
    if (getRightOfWayEntities(following_lanelets).size() != 0) {
      return BT::NodeStatus::FAILURE;
    }
    auto distance_to_front_entity = getDistanceToFrontEntity();
    if (distance_to_front_entity) {
      if (distance_to_front_entity.get() <=
        calculateStopDistance() +
        vehicle_parameters->bounding_box.dimensions.length + 5)
      {
        return BT::NodeStatus::FAILURE;
      }
    }
    auto distance_to_stopline = getDistanceToStopLine(following_lanelets);
    auto distance_to_conflicting_entity = getDistanceToConflictingEntity(following_lanelets);
    if (distance_to_stopline) {
      if (distance_to_stopline.get() <=
        calculateStopDistance() +
        vehicle_parameters->bounding_box.dimensions.length * 0.5 + 5)
      {
        return BT::NodeStatus::FAILURE;
      }
    }
    if (distance_to_conflicting_entity) {
      if (distance_to_conflicting_entity.get() <
        (vehicle_parameters->bounding_box.dimensions.length + calculateStopDistance() + 10))
      {
        return BT::NodeStatus::FAILURE;
      }
    }
    if (!target_speed) {
      target_speed = hdmap_utils->getSpeedLimit(following_lanelets);
    }
    auto updated_status = calculateEntityStatusUpdated(target_speed.get());
    setOutput("updated_status", updated_status);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
