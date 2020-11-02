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

#include <simulation_api/behavior/vehicle/vehicle_action_node.hpp>

#include <string>
#include <memory>
#include <vector>
#include <utility>

namespace entity_behavior
{
VehicleActionNode::VehicleActionNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: ActionNode(name, config) {}

void VehicleActionNode::getBlackBoardValues()
{
  ActionNode::getBlackBoardValues();
  if (!getInput<std::shared_ptr<simulation_api::entity::VehicleParameters>>(
      "vehicle_parameters", vehicle_parameters))
  {
    throw BehaviorTreeRuntimeError("failed to get input vehicle_parameters in VehicleActionNode");
  }
}

simulation_api::entity::EntityStatus VehicleActionNode::calculateEntityStatusUpdated(
  double target_speed,
  const std::vector<int> & following_lanelets) const
{
  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::WORLD) {
    throw BehaviorTreeRuntimeError(
            "entity status must be lane when we call calculateEntityStatusUpdated");
  }
  geometry_msgs::msg::Accel accel_new;
  accel_new = entity_status.accel;
  double target_accel = (target_speed - entity_status.twist.linear.x) / step_time;
  if (entity_status.twist.linear.x > target_speed) {
    target_accel = boost::algorithm::clamp(target_accel, -5, 0);
  } else {
    target_accel = boost::algorithm::clamp(target_accel, 0, 3);
  }
  accel_new.linear.x = target_accel;
  geometry_msgs::msg::Twist twist_new;
  twist_new.linear.x = boost::algorithm::clamp(
    entity_status.twist.linear.x + accel_new.linear.x * step_time,
    0, vehicle_parameters->performance.max_speed);
  twist_new.linear.y = 0.0;
  twist_new.linear.z = 0.0;
  twist_new.angular.x = 0.0;
  twist_new.angular.y = 0.0;
  twist_new.angular.z = 0.0;
  int new_lanelet_id = entity_status.lanelet_id;
  double new_s = entity_status.s + (twist_new.linear.x + entity_status.twist.linear.x) / 2.0 *
    step_time;

  bool calculation_success = false;
  for (size_t i = 0; i < following_lanelets.size(); i++) {
    if (following_lanelets[i] == entity_status.lanelet_id) {
      double length = hdmap_utils->getLaneletLength(entity_status.lanelet_id);
      calculation_success = true;
      if (length < new_s) {
        if (i != (following_lanelets.size() - 1)) {
          new_s = new_s - length;
          new_lanelet_id = following_lanelets[i + 1];
          break;
        } else {
          new_s = new_s - length;
          auto next_ids = hdmap_utils->getNextLaneletIds(following_lanelets[i]);
          if (next_ids.size() == 0) {
            throw BehaviorTreeRuntimeError(
                    "failed to get next lane in calculateEntityStatusUpdated function");
          }
          new_lanelet_id = next_ids[0];
          break;
        }
      }
    }
  }
  if (!calculation_success) {
    throw BehaviorTreeRuntimeError(
            "failed to calculate next status calculateEntityStatusUpdated function");
  }
  simulation_api::entity::EntityStatus entity_status_updated(current_time + step_time,
    new_lanelet_id, new_s, entity_status.offset, entity_status.rpy, twist_new, accel_new);
  return entity_status_updated;
}

simulation_api::entity::EntityStatus VehicleActionNode::calculateEntityStatusUpdated(
  double target_speed) const
{
  geometry_msgs::msg::Accel accel_new;
  double target_accel = (target_speed - entity_status.twist.linear.x) / step_time;
  if (entity_status.twist.linear.x > target_speed) {
    target_accel = boost::algorithm::clamp(target_accel, -5, 0);
  } else {
    target_accel = boost::algorithm::clamp(target_accel, 0, 3);
  }
  accel_new.linear.x = target_accel;
  geometry_msgs::msg::Twist twist_new;
  twist_new.linear.x = boost::algorithm::clamp(
    entity_status.twist.linear.x + accel_new.linear.x * step_time,
    0, vehicle_parameters->performance.max_speed);
  twist_new.linear.y = 0.0;
  twist_new.linear.z = 0.0;
  twist_new.angular.x = 0.0;
  twist_new.angular.y = 0.0;
  twist_new.angular.z = 0.0;

  double new_s = entity_status.s + (twist_new.linear.x + entity_status.twist.linear.x) / 2.0 *
    step_time;
  simulation_api::entity::EntityStatus entity_status_updated(current_time + step_time,
    entity_status.lanelet_id, new_s, entity_status.offset, entity_status.rpy, twist_new, accel_new);
  return entity_status_updated;
}
}  // namespace entity_behavior
