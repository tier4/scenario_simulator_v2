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

#ifndef SIMULATION_CONTROLLER__BEHAVIOR__VEHICLE__VEHICLE_ACTION_NODE_HPP_
#define SIMULATION_CONTROLLER__BEHAVIOR__VEHICLE__VEHICLE_ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <simulation_controller/behavior/action_node.hpp>
#include <string>

namespace entity_behavior
{
class VehicleActionNode : public ActionNode
{
public:
  VehicleActionNode(const std::string & name, const BT::NodeConfiguration & config);
  ~VehicleActionNode() override = default;
};
}  // namespace entity_behavior

#endif  // SIMULATION_CONTROLLER__BEHAVIOR__VEHICLE__VEHICLE_ACTION_NODE_HPP_
