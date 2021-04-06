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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__PEDESTRIAN__PEDESTRIAN_ACTION_NODE_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__PEDESTRIAN__PEDESTRIAN_ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <traffic_simulator/entity/pedestrian_parameter.hpp>
#include <traffic_simulator/behavior/action_node.hpp>

#include <openscenario_msgs/msg/pedestrian_parameters.hpp>

#include <string>
#include <memory>

namespace entity_behavior
{
class PedestrianActionNode : public ActionNode
{
public:
  PedestrianActionNode(const std::string & name, const BT::NodeConfiguration & config);
  void getBlackBoardValues();
  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {
      BT::InputPort<openscenario_msgs::msg::PedestrianParameters>(
        "pedestrian_parameters")
    };
    BT::PortsList parent_ports = entity_behavior::ActionNode::providedPorts();
    for (const auto & parent_port : parent_ports) {
      ports.emplace(parent_port.first, parent_port.second);
    }
    return ports;
  }
  openscenario_msgs::msg::PedestrianParameters pedestrian_parameters;
  openscenario_msgs::msg::EntityStatus calculateEntityStatusUpdatedInWorldFrame(
    double target_speed);
  openscenario_msgs::msg::EntityStatus calculateEntityStatusUpdated(
    double target_speed);
};
}  // namespace entity_behavior

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__PEDESTRIAN__PEDESTRIAN_ACTION_NODE_HPP_
