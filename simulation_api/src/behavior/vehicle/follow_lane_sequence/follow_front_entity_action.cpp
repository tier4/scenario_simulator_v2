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
#include <simulation_api/behavior/vehicle/follow_lane_sequence/follow_front_entity_action.hpp>

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

BT::NodeStatus FollowFrontEntityAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::WORLD) {
    return BT::NodeStatus::FAILURE;
  }
  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::LANE) {
    boost::optional<double> front_entity_distance, front_entity_speed;
    for (const auto & each : other_entity_status) {
      if (each.second.coordinate == simulation_api::entity::CoordinateFrameTypes::LANE) {
        auto distance = hdmap_utils->getLongitudinalDistance(entity_status.lanelet_id,
            entity_status.s,
            each.second.lanelet_id,
            each.second.s);
        if (distance) {
          if (distance.get() < 40) {
            if (!front_entity_distance && !front_entity_speed) {
              front_entity_speed = each.second.twist.linear.x;
              front_entity_distance = distance.get();
            } else {
              if (front_entity_distance.get() < distance.get()) {
                front_entity_speed = each.second.twist.linear.x;
                front_entity_distance = distance.get();
              }
            }
          }
        }
      }
    }
    if (!front_entity_distance && !front_entity_speed) {
      return BT::NodeStatus::SUCCESS;
    }
    auto entity_status_updated = calculateEntityStatusUpdated(front_entity_speed.get());
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  }
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
