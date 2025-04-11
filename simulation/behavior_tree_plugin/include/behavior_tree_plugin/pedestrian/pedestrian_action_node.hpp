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

#ifndef BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__PEDESTRIAN_ACTION_NODE_HPP_
#define BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__PEDESTRIAN_ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>

#include <behavior_tree_plugin/action_node.hpp>
#include <memory>
#include <string>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>

namespace entity_behavior
{
class PedestrianActionNode : public ActionNode
{
public:
  PedestrianActionNode(const std::string & name, const BT::NodeConfiguration & config);
  void getBlackBoardValues() override;
  static BT::PortsList providedPorts()
  {
    // clang-format off
    return BT::PortsList({
      BT::InputPort<traffic_simulator_msgs::msg::PedestrianParameters>("pedestrian_parameters"),
    }) + entity_behavior::ActionNode::providedPorts();
    // clang-format on
  }
  traffic_simulator_msgs::msg::PedestrianParameters pedestrian_parameters;
  auto calculateUpdatedEntityStatusInWorldFrame(double target_speed) const
    -> traffic_simulator::EntityStatus;
  auto calculateUpdatedEntityStatus(double target_speed) const -> traffic_simulator::EntityStatus;
};
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__PEDESTRIAN_ACTION_NODE_HPP_
