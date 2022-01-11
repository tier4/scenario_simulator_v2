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

#include <behavior_tree_plugin/transition_events/reset_request_event.hpp>

namespace behavior_tree_plugin
{
ResetRequestEvent::ResetRequestEvent(
  BT::TreeNode * root_node, std::function<std::string()> get_request_function,
  std::function<void(const std::string &)> set_request_function)
: TransitionEvent(root_node),
  get_request_function_(get_request_function),
  set_request_function_(set_request_function)
{
}

void ResetRequestEvent::callback(
  BT::Duration /*timestamp*/, const BT::TreeNode & node, BT::NodeStatus /*prev_status*/,
  BT::NodeStatus status)
{
  TransitionEvent::updateCurrentAction(status, node);
  if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
    if (get_request_function_() == current_action_) {
      set_request_function_("none");
    }
  }
}
}  // namespace behavior_tree_plugin
