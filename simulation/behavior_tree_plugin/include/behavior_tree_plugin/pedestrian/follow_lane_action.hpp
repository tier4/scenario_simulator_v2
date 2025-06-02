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

#ifndef BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__FOLLOW_LANE_ACTION_HPP_
#define BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__FOLLOW_LANE_ACTION_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <behavior_tree_plugin/pedestrian/pedestrian_action_node.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/operator.hpp>
#include <get_parameter/get_parameter.hpp>
#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <vector>

namespace entity_behavior
{
namespace pedestrian
{
enum class SeeAroundMode { blind, aware };

class FollowLaneAction : public entity_behavior::PedestrianActionNode
{
public:
  FollowLaneAction(const std::string & name, const BT::NodeConfiguration & config);
  bool checkPreconditions() override;
  BT::NodeStatus doAction() override;
  void getBlackBoardValues() override;
  static BT::PortsList providedPorts()
  {
    return entity_behavior::PedestrianActionNode::providedPorts();
  }
  bool detectObstacleInLane(const lanelet::Ids pedestrian_lanes, const bool see_around) const;

private:
  SeeAroundMode should_respect_see_around;
};
}  // namespace pedestrian
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__FOLLOW_LANE_ACTION_HPP_
