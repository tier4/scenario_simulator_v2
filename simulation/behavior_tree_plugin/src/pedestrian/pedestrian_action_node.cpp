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

#include <behavior_tree_plugin/pedestrian/pedestrian_action_node.hpp>
#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/helper/helper.hpp>

namespace entity_behavior
{
PedestrianActionNode::PedestrianActionNode(
  const std::string & name, const BT::NodeConfiguration & config)
: ActionNode(name, config)
{
}

void PedestrianActionNode::getBlackBoardValues()
{
  ActionNode::getBlackBoardValues();
  if (!getInput<traffic_simulator_msgs::msg::BehaviorParameter>(
        "behavior_parameter", behavior_parameter)) {
    behavior_parameter = traffic_simulator_msgs::msg::BehaviorParameter();
  }
  if (!getInput<traffic_simulator_msgs::msg::PedestrianParameters>(
        "pedestrian_parameters", pedestrian_parameters)) {
    THROW_SIMULATION_ERROR("failed to get input pedestrian_parameters in PedestrianActionNode");
  }
}

traffic_simulator_msgs::msg::EntityStatus PedestrianActionNode::calculateEntityStatusUpdated(
  double target_speed)
{
  return ActionNode::calculateEntityStatusUpdated(
    target_speed, 10, behavior_parameter.dynamic_constraints);
}
}  // namespace entity_behavior
