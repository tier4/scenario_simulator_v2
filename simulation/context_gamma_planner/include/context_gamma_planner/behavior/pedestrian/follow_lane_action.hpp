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

#include "context_gamma_planner/behavior/pedestrian/action_node.hpp"
#include "context_gamma_planner/planner/pedestrian/follow_lane_planner.hpp"

namespace context_gamma_planner::pedestrian
{
class FollowLaneAction : public context_gamma_planner::pedestrian::ActionNode
{
public:
  FollowLaneAction(const std::string & name, const BT::NodeConfiguration & config);

  BT::NodeStatus tick() override;

  void getBlackBoardValues() override;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {};
    BT::PortsList parent_ports = context_gamma_planner::pedestrian::ActionNode::providedPorts();
    for (const auto & [port_name, port_info] : parent_ports) {
      ports.try_emplace(port_name, port_info);
    }
    return ports;
  }

private:
  FollowLanePlanner planner_{3.0};
};
}  // namespace context_gamma_planner::pedestrian

#endif  // CONTEXT_GAMMA_PLANNER__BEHAVIOR__PEDESTRIAN__FOLLOW_LANE_ACTION_HPP_
