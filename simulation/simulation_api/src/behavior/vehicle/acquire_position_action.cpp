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
#include <simulation_api/behavior/vehicle/acquire_position_action.hpp>

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

void AcquirePositionAction::getBlackBoardValues()
{
  simulation_api::entity::EntityStatus target_status;
  VehicleActionNode::getBlackBoardValues();
  if (!getInput<simulation_api::entity::EntityStatus>("target_status", target_status)) {
    target_status_ = boost::none;
    route_ = boost::none;
  } else {
    target_status_ = target_status;
  }
}

BT::NodeStatus AcquirePositionAction::tick()
{
  getBlackBoardValues();
  if (request != "acquire_position") {
    route_ = boost::none;
    target_status_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }

  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::WORLD) {
    route_ = boost::none;
    target_status_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }

  if (target_status_->coordinate == simulation_api::entity::CoordinateFrameTypes::WORLD) {
    route_ = boost::none;
    target_status_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }

  route_ = hdmap_utils->getRoute(entity_status.lanelet_id, target_status_->lanelet_id);

  if (!target_speed) {
    std::vector<int> following_lanelets;
    bool is_finded = false;
    for (auto itr = route_->begin(); itr != route_->end(); itr++) {
      if (is_finded) {
        if (following_lanelets.size() <= 3) {
          following_lanelets.push_back(*itr);
        }
      } else {
        if (entity_status.lanelet_id == *itr) {
          following_lanelets.push_back(*itr);
          is_finded = true;
        }
      }
    }
    if (following_lanelets.size() != 0) {
      target_speed = hdmap_utils->getSpeedLimit(following_lanelets);
    }
  }
  auto distance_to_front_entity = getDistanceToFrontEntity();
  if (distance_to_front_entity) {
    if (distance_to_front_entity.get() <=
      calculateStopDistance() +
      vehicle_parameters->bounding_box.dimensions.length + 5)
    {
      auto front_entity_status = getFrontEntityStatus();
      if (front_entity_status) {
        target_speed = front_entity_status->twist.linear.x;
      }
    }
  }
  auto entity_status_updated = calculateEntityStatusUpdated(target_speed.get(), route_.get());
  setOutput("updated_status", entity_status_updated);
  if (target_status_->lanelet_id == entity_status.lanelet_id) {
    if (target_status_->s < entity_status.s) {
      route_ = boost::none;
      target_status_ = boost::none;
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::RUNNING;
}
}      // namespace vehicle
}  // namespace entity_behavior
