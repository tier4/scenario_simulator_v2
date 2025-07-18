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

#ifndef CONTEXT_GAMMA_PLANNER__BEHAVIOR__PEDESTRIAN__FOLLOW_LANE_ACTION_HPP_
#define CONTEXT_GAMMA_PLANNER__BEHAVIOR__PEDESTRIAN__FOLLOW_LANE_ACTION_HPP_

#include <context_gamma_planner/behavior/pedestrian/action_node.hpp>
#include <context_gamma_planner/planner/pedestrian/follow_lane_planner.hpp>

namespace context_gamma_planner
{
namespace pedestrian
{
class FollowLaneAction : public context_gamma_planner::pedestrian::ActionNode
{
public:
  /**
   * @brief Represents an action node in the behavior tree.
   * @param name The name of the action node.
   * @param config The configuration for the action node.
   */
  FollowLaneAction(const std::string & name, const BT::NodeConfiguration & config);

  /**
   * @brief update function
   * @return node status : RUNNING, SUCCESS, FAILURE, and IDLE.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief This function retrieves the values stored in the blackboard and performs any necessary operations.
   */
  void getBlackBoardValues();

  /**
   * @brief A list of ports for a behavior tree node.
   */
  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {};
    BT::PortsList parent_ports = context_gamma_planner::pedestrian::ActionNode::providedPorts();
    for (const auto & parent_port : parent_ports) {
      ports.emplace(parent_port.first, parent_port.second);
    }
    return ports;
  }

private:
  FollowLanePlanner planner_;
};
}  // namespace pedestrian
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__BEHAVIOR__PEDESTRIAN__FOLLOW_LANE_ACTION_HPP_
