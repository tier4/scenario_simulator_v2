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

#ifndef BEHAVIOR_TREE_PLUGIN__TRANSITION_EVENTS__LOGGING_EVENT_HPP_
#define BEHAVIOR_TREE_PLUGIN__TRANSITION_EVENTS__LOGGING_EVENT_HPP_

#include <behavior_tree_plugin/transition_events/transition_event.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace behavior_tree_plugin
{
class LoggingEvent : public TransitionEvent
{
public:
  LoggingEvent(BT::TreeNode * root_node, const rclcpp::Logger & logger);
  const std::string & getCurrentAction() const;

private:
  void callback(
    BT::Duration timestamp, const BT::TreeNode & node, BT::NodeStatus prev_status,
    BT::NodeStatus status) override;
  rclcpp::Logger ros_logger_;
};
}  // namespace behavior_tree_plugin

#endif  // BEHAVIOR_TREE_PLUGIN__TRANSITION_EVENTS__LOGGING_EVENT_HPP_
