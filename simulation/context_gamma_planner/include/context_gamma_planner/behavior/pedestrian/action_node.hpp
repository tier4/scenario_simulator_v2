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

#ifndef CONTEXT_GAMMA_PLANNER__PEDESTRIAN__PEDESTRIAN_ACTION_NODE_HPP_
#define CONTEXT_GAMMA_PLANNER__PEDESTRIAN__PEDESTRIAN_ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>

#include <context_gamma_planner/behavior/action_node_base.hpp>
#include <context_gamma_planner/constraints/pedestrian/constraint_activator.hpp>
#include <memory>
#include <string>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>

namespace context_gamma_planner
{
namespace pedestrian
{
class ActionNode : public ActionNodeBase
{
public:
  /**
   * @brief Represents an action node in the behavior tree.
   * @param name The name of the action node.
   * @param config The configuration for the action node.
   */
  ActionNode(const std::string & name, const BT::NodeConfiguration & config);
  void getBlackBoardValues();

  /**
   * @brief A list of ports for a behavior tree node.
   */
  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {
      BT::InputPort<traffic_simulator_msgs::msg::PedestrianParameters>("pedestrian_parameters")};
    BT::PortsList parent_ports = context_gamma_planner::ActionNodeBase::providedPorts();
    for (const auto & parent_port : parent_ports) {
      ports.emplace(parent_port.first, parent_port.second);
    }
    return ports;
  }
  traffic_simulator_msgs::msg::PedestrianParameters pedestrian_parameters;
};
}  // namespace pedestrian
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__PEDESTRIAN__PEDESTRIAN_ACTION_NODE_HPP_
