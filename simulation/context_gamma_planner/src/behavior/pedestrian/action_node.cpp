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

#include <context_gamma_planner/behavior/pedestrian/action_node.hpp>
#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <string>

namespace context_gamma_planner
{
namespace pedestrian
{
ActionNode::ActionNode(const std::string & name, const BT::NodeConfiguration & config)
: ActionNodeBase(name, config)
{
}

void ActionNode::getBlackBoardValues()
{
  ActionNodeBase::getBlackBoardValues();
  if (!getInput<traffic_simulator_msgs::msg::PedestrianParameters>(
        "pedestrian_parameters", pedestrian_parameters)) {
    THROW_SIMULATION_ERROR("failed to get input pedestrian_parameters in pedestrian::ActionNode");
  }
  getInput("activator", activator);
  if (activator == nullptr) {
    THROW_SIMULATION_ERROR("constraint activator is nullptr");
  }
}
}  // namespace pedestrian
}  // namespace context_gamma_planner
