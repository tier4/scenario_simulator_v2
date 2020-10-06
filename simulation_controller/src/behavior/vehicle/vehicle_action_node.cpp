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

#include <simulation_controller/behavior/vehicle/vehicle_action_node.hpp>
#include <string>

namespace entity_behavior
{
VehicleActionNode::VehicleActionNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: ActionNode(name, config) {}

void VehicleActionNode::getBlackBoardValues()
{
  ActionNode::getBlackBoardValues();
  if (!getInput<std::shared_ptr<simulation_controller::entity::VehicleParameters>>(
      "vehicle_parameters", vehicle_parameters))
  {
    throw BehaviorTreeRuntimeError("failed to get input vehicle_parameters in VehicleActionNode");
  }
}
}
