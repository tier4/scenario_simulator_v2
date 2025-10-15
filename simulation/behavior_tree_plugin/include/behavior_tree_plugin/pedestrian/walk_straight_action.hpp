/**
 * @file walk_straight_action.hpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief class definition for the walk straight action
 * @version 0.1
 * @date 2021-04-02
 *
 * @copyright Copyright(c) TIER IV.Inc {2015}
 *
 */

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

#ifndef BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__WALK_STRAIGHT_ACTION_HPP_
#define BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__WALK_STRAIGHT_ACTION_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <behavior_tree_plugin/pedestrian/pedestrian_action_node.hpp>
#include <memory>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <vector>

namespace entity_behavior
{
namespace pedestrian
{
class WalkStraightAction : public entity_behavior::PedestrianActionNode
{
public:
  WalkStraightAction(const std::string & name, const BT::NodeConfiguration & config);
  bool checkPreconditions() override;
  BT::NodeStatus doAction() override;
  void getBlackBoardValues() override;
  static BT::PortsList providedPorts()
  {
    return entity_behavior::PedestrianActionNode::providedPorts();
  }

private:
  bool isObstacleInFront(const bool see_around) const;
  bool isEntityColliding(
    const traffic_simulator::entity_status::CanonicalizedEntityStatus & entity_status,
    const double & detection_horizon) const;
  auto calculateWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray;

  static constexpr double front_entity_margin = 2.0;
};
}  // namespace pedestrian
}  // namespace entity_behavior

#endif  // BEHAVIOR_TREE_PLUGIN__PEDESTRIAN__WALK_STRAIGHT_ACTION_HPP_
