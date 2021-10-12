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

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behavior_tree_plugin/vehicle/behavior_tree.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/follow_front_entity_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/follow_lane_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/move_backward_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/stop_at_crossing_entity_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/stop_at_stop_line_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/stop_at_traffic_light_action.hpp>
#include <behavior_tree_plugin/vehicle/follow_lane_sequence/yield_action.hpp>
#include <behavior_tree_plugin/vehicle/lane_change_action.hpp>
#include <iostream>
#include <openscenario_msgs/msg/driver_model.hpp>
#include <string>
#include <utility>

namespace entity_behavior
{
namespace vehicle
{
BehaviorTree::BehaviorTree()
{
  std::string path = ament_index_cpp::get_package_share_directory("behavior_tree_plugin") +
                     "/config/vehicle_entity_behavior.xml";
  factory_.registerNodeType<follow_lane_sequence::FollowLaneAction>("FollowLane");
  factory_.registerNodeType<follow_lane_sequence::FollowFrontEntityAction>("FollowFrontEntity");
  factory_.registerNodeType<follow_lane_sequence::StopAtCrossingEntityAction>(
    "StopAtCrossingEntity");
  factory_.registerNodeType<follow_lane_sequence::StopAtStopLineAction>("StopAtStopLine");
  factory_.registerNodeType<follow_lane_sequence::StopAtTrafficLightAction>("StopAtTrafficLight");
  factory_.registerNodeType<follow_lane_sequence::YieldAction>("Yield");
  factory_.registerNodeType<follow_lane_sequence::MoveBackwardAction>("MoveBackward");
  factory_.registerNodeType<LaneChangeAction>("LaneChange");
  tree_ = factory_.createTreeFromFile(path);
  configure();
}
}  // namespace vehicle
}  // namespace entity_behavior

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(entity_behavior::vehicle::BehaviorTree, entity_behavior::BehaviorPluginBase)
