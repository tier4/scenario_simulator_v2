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
#include <string>
#include <traffic_simulator_msgs/msg/driver_model.hpp>
#include <utility>

namespace entity_behavior
{
void VehicleBehaviorTree::configure(const rclcpp::Logger & logger)
{
  std::string path = ament_index_cpp::get_package_share_directory("behavior_tree_plugin") +
                     "/config/vehicle_entity_behavior.xml";
  factory_.registerNodeType<entity_behavior::vehicle::follow_lane_sequence::FollowLaneAction>(
    "FollowLane");
  factory_
    .registerNodeType<entity_behavior::vehicle::follow_lane_sequence::FollowFrontEntityAction>(
      "FollowFrontEntity");
  factory_
    .registerNodeType<entity_behavior::vehicle::follow_lane_sequence::StopAtCrossingEntityAction>(
      "StopAtCrossingEntity");
  factory_.registerNodeType<entity_behavior::vehicle::follow_lane_sequence::StopAtStopLineAction>(
    "StopAtStopLine");
  factory_
    .registerNodeType<entity_behavior::vehicle::follow_lane_sequence::StopAtTrafficLightAction>(
      "StopAtTrafficLight");
  factory_.registerNodeType<entity_behavior::vehicle::follow_lane_sequence::YieldAction>("Yield");
  factory_.registerNodeType<entity_behavior::vehicle::follow_lane_sequence::MoveBackwardAction>(
    "MoveBackward");
  factory_.registerNodeType<entity_behavior::vehicle::LaneChangeAction>("LaneChange");
  tree_ = factory_.createTreeFromFile(path);

  logging_event_ptr_ =
    std::make_unique<behavior_tree_plugin::LoggingEvent>(tree_.rootNode(), logger);
  reset_request_event_ptr_ = std::make_unique<behavior_tree_plugin::ResetRequestEvent>(
    tree_.rootNode(), [&]() { return getRequest(); },
    [&](std::string request) { return setRequest(request); });
  setRequest("none");
}

const std::string & VehicleBehaviorTree::getCurrentAction() const
{
  return logging_event_ptr_->getCurrentAction();
}

void VehicleBehaviorTree::update(double current_time, double step_time)
{
  tickOnce(current_time, step_time);
  while (getCurrentAction() == "root") {
    tickOnce(current_time, step_time);
  }
}

BT::NodeStatus VehicleBehaviorTree::tickOnce(double current_time, double step_time)
{
  setCurrentTime(current_time);
  setStepTime(step_time);
  const auto ret = tree_.rootNode()->executeTick();
  return ret;
}
}  // namespace entity_behavior

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(entity_behavior::VehicleBehaviorTree, entity_behavior::BehaviorPluginBase)
