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

#ifndef CONTEXT_GAMMA_PLANNER__PEDESTRIAN__FOLLOW_POLYLINE_TRAJECTORY_ACTION_HPP_
#define CONTEXT_GAMMA_PLANNER__PEDESTRIAN__FOLLOW_POLYLINE_TRAJECTORY_ACTION_HPP_

#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/helper/stop_watch.hpp>
#include <vector>

#include "context_gamma_planner/behavior/pedestrian/action_node.hpp"
#include "context_gamma_planner/planner/pedestrian/follow_polyline_trajectory_planner.hpp"

namespace context_gamma_planner::pedestrian
{

class FollowPolylineTrajectoryAction : public context_gamma_planner::pedestrian::ActionNode
{
public:
  FollowPolylineTrajectoryAction(const std::string & name, const BT::NodeConfiguration & config);

  BT::NodeStatus tick() override;

  void getBlackBoardValues();

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {
      BT::InputPort<std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory>>(
        "polyline_trajectory")};
    BT::PortsList parent_ports = context_gamma_planner::pedestrian::ActionNode::providedPorts();
    for (const auto & parent_port : parent_ports) {
      ports.emplace(parent_port.first, parent_port.second);
    }
    return ports;
  }

private:
  std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> polyline_trajectory_;
  FollowPolylineTrajectoryPlanner planner_;
};
}  // namespace context_gamma_planner::pedestrian

#endif  // CONTEXT_GAMMA_PLANNER__PEDESTRIAN__FOLLOW_POLYLINE_TRAJECTORY_ACTION_HPP_
