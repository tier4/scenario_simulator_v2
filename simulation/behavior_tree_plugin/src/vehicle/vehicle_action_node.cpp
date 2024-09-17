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

#include <behavior_tree_plugin/vehicle/vehicle_action_node.hpp>
#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <utility>
#include <vector>

namespace entity_behavior
{
VehicleActionNode::VehicleActionNode(const std::string & name, const BT::NodeConfiguration & config)
: ActionNode(name, config)
{
}

void VehicleActionNode::getBlackBoardValues()
{
  ActionNode::getBlackBoardValues();
  if (!getInput<traffic_simulator_msgs::msg::BehaviorParameter>(
        "behavior_parameter", behavior_parameter)) {
    behavior_parameter = traffic_simulator_msgs::msg::BehaviorParameter();
  }
  if (!getInput<traffic_simulator_msgs::msg::VehicleParameters>(
        "vehicle_parameters", vehicle_parameters)) {
    THROW_SIMULATION_ERROR("failed to get input vehicle_parameters in VehicleActionNode");
  }
  if (!getInput<std::shared_ptr<math::geometry::CatmullRomSpline>>(
        "reference_trajectory", reference_trajectory)) {
    THROW_SIMULATION_ERROR("failed to get input reference_trajectory in VehicleActionNode");
  }
}

auto VehicleActionNode::calculateUpdatedEntityStatus(double target_speed) const
  -> traffic_simulator::EntityStatus
{
  return ActionNode::calculateUpdatedEntityStatus(
    target_speed, behavior_parameter.dynamic_constraints);
}

auto VehicleActionNode::calculateUpdatedEntityStatusInWorldFrame(double target_speed) const
  -> traffic_simulator::EntityStatus
{
  if (target_speed > vehicle_parameters.performance.max_speed) {
    target_speed = vehicle_parameters.performance.max_speed;
  } else {
    target_speed = canonicalized_entity_status->getTwist().linear.x;
  }
  return ActionNode::calculateUpdatedEntityStatusInWorldFrame(
    target_speed, behavior_parameter.dynamic_constraints);
}
}  // namespace entity_behavior
