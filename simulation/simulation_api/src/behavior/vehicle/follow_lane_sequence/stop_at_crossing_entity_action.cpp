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
#include <simulation_api/behavior/vehicle/follow_lane_sequence/stop_at_crossing_entity_action.hpp>
#include <simulation_api/math/catmull_rom_spline.hpp>

#include <string>
#include <vector>
#include <utility>

namespace entity_behavior
{
namespace vehicle
{
namespace follow_lane_sequence
{
StopAtCrossingEntityAction::StopAtCrossingEntityAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config) {}

const openscenario_msgs::msg::WaypointsArray StopAtCrossingEntityAction::calculateWaypoints()
{
  return openscenario_msgs::msg::WaypointsArray();
}

boost::optional<double> StopAtCrossingEntityAction::calculateTargetSpeed(
  const std::vector<std::int64_t> & following_lanelets, double current_velocity)
{
  auto distance_to_stop_target = getDistanceToConflictingEntity(following_lanelets);
  if (!distance_to_stop_target) {
    return boost::none;
  }
  double rest_distance = distance_to_stop_target.get() -
    (vehicle_parameters->bounding_box.dimensions.length + 10);
  if (rest_distance < calculateStopDistance()) {
    if (rest_distance > 0) {
      return std::sqrt(2 * 5 * rest_distance);
    } else {
      return 0;
    }
  }
  return current_velocity;
}

BT::NodeStatus StopAtCrossingEntityAction::tick()
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
  auto target_linear_speed =
    calculateTargetSpeed(following_lanelets, entity_status.action_status.twist.linear.x);
  if (!target_linear_speed) {
    setOutput("updated_status", calculateEntityStatusUpdated(0));
    return BT::NodeStatus::SUCCESS;
  }
  if (target_speed) {
    if (target_speed.get() > target_linear_speed.get()) {
      target_speed = target_linear_speed.get();
    }
  } else {
    target_speed = target_linear_speed.get();
  }
  setOutput("updated_status", calculateEntityStatusUpdated(target_speed.get()));
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
