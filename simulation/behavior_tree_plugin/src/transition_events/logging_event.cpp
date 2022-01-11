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

#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <behavior_tree_plugin/transition_events/logging_event.hpp>

namespace behavior_tree_plugin
{
LoggingEvent::LoggingEvent(BT::TreeNode * root_node, const rclcpp::Logger & logger)
: TransitionEvent(root_node), ros_logger_(logger)
{
}

void LoggingEvent::callback(
  BT::Duration /*timestamp*/, const BT::TreeNode & node, BT::NodeStatus prev_status,
  BT::NodeStatus status)
{
  RCLCPP_INFO_STREAM(
    ros_logger_, "Action " << node.name() << " changed status, " << BT::toStr(prev_status, true)
                           << " => " << BT::toStr(status, true));
  TransitionEvent::updateCurrentAction(status, node);
}

const std::string & LoggingEvent::getCurrentAction() const { return current_action_; }
}  // namespace behavior_tree_plugin
