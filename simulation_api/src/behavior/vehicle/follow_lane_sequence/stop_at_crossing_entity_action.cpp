// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

boost::optional<double> StopAtCrossingEntityAction::calculateTargetSpeed(
  const std::vector<int> & following_lanelets)
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(following_lanelets);
  std::vector<simulation_api::entity::EntityStatus> conflicting_entity_status;
  for (const auto & status : other_entity_status) {
    if (status.second.coordinate == simulation_api::entity::CoordinateFrameTypes::LANE) {
      if (std::count(conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        status.second.lanelet_id) >= 1)
      {
        conflicting_entity_status.push_back(status.second);
      }
    }
  }
  std::vector<double> dists;
  std::vector<std::pair<int, double>> collision_points;
  for (const auto & status : conflicting_entity_status) {
    for (const auto & lanelet_id : following_lanelets) {
      auto stop_position_s = hdmap_utils->getCollisionPointInLaneCoordinate(lanelet_id,
          status.lanelet_id);
      if (stop_position_s) {
        auto dist = hdmap_utils->getLongitudinalDistance(entity_status.lanelet_id,
            entity_status.s,
            lanelet_id, stop_position_s.get());
        if (dist) {
          dists.push_back(dist.get());
          collision_points.push_back(std::make_pair(lanelet_id, stop_position_s.get()));
        }
      }
    }
  }
  if (dists.size() != 0) {
    auto iter = std::max_element(dists.begin(), dists.end());
    size_t index = std::distance(dists.begin(), iter);
    double stop_s = collision_points[index].second;
    int stop_lanelet_id = collision_points[index].first;
    geometry_msgs::msg::Vector3 rpy;
    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Accel accel;
    simulation_api::entity::EntityStatus stop_target_status(0.0, stop_lanelet_id,
      stop_s, 0, rpy, twist, accel);
    auto dist_to_stop_target = hdmap_utils->getLongitudinalDistance(
      entity_status.lanelet_id, entity_status.s,
      stop_target_status.lanelet_id, stop_target_status.s);
    if (dist_to_stop_target) {
      double rest_distance = dist_to_stop_target.get() -
        (vehicle_parameters->bounding_box.dimensions.length + 5);
      if (rest_distance < std::pow(entity_status.twist.linear.x, 2) / (2 * 5)) {
        if (rest_distance > 0) {
          return std::sqrt(2 * 5 * rest_distance);
        } else {
          return 0;
        }
      }
    }
  }
  return boost::none;
}

BT::NodeStatus StopAtCrossingEntityAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::WORLD) {
    return BT::NodeStatus::FAILURE;
  }
  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::LANE) {
    auto following_lanelets = hdmap_utils->getFollowingLanelets(entity_status.lanelet_id, 50);
    auto target_linear_speed = calculateTargetSpeed(following_lanelets);
    if (!target_linear_speed) {
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
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
