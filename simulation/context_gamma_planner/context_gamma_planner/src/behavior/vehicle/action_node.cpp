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

#include <context_gamma_planner/behavior/vehicle/action_node.hpp>
#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <utility>
#include <vector>

namespace context_gamma_planner
{
namespace vehicle
{
ActionNode::ActionNode(const std::string & name, const BT::NodeConfiguration & config)
: ActionNodeBase(name, config)
{
}

void ActionNode::getBlackBoardValues()
{
  ActionNodeBase::getBlackBoardValues();
  if (!getInput<traffic_simulator_msgs::msg::BehaviorParameter>(
        "behavior_parameter", behavior_parameter)) {
    behavior_parameter = traffic_simulator_msgs::msg::BehaviorParameter();
  }
  if (!getInput<traffic_simulator_msgs::msg::VehicleParameters>(
        "vehicle_parameters", vehicle_parameters)) {
    THROW_SIMULATION_ERROR("failed to get input vehicle_parameters in vehicle::ActionNode");
  }
  if (!getInput<std::shared_ptr<math::geometry::CatmullRomSpline>>(
        "reference_trajectory", reference_trajectory)) {
    THROW_SIMULATION_ERROR("failed to get input reference_trajectory in vehicle::ActionNode");
  }
  getInput("activator", activator);
  if (activator == nullptr) {
    THROW_SIMULATION_ERROR("constraint activator is nullptr");
  }
}
}  // namespace vehicle
}  // namespace context_gamma_planner
