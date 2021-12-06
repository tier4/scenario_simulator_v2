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

#include <behavior_tree_plugin/transition_events/transition_event.hpp>

namespace behavior_tree_plugin
{
TransitionEvent::TransitionEvent(const std::shared_ptr<BT::TreeNode> & root_node)
: root_node_(root_node)
{
  first_timestamp_ = std::chrono::high_resolution_clock::now();
  auto subscribeCallback = [this](
                             BT::TimePoint timestamp, const BT::TreeNode & node,
                             BT::NodeStatus prev, BT::NodeStatus status) {
    if (status != BT::NodeStatus::IDLE) {
#if FOXY
      if (type_ == BT::TimestampType::absolute) {
#else
      if (type_ == BT::TimestampType::ABSOLUTE) {
#endif
        this->callback(timestamp.time_since_epoch(), node, prev, status);
      } else {
        this->callback(timestamp - first_timestamp_, node, prev, status);
      }
    }
  };
  auto visitor = [this, subscribeCallback](BT::TreeNode * node) {
    subscribers_.push_back(node->subscribeToStatusChange(std::move(subscribeCallback)));
  };
  BT::applyRecursiveVisitor(root_node.get(), visitor);
}

void TransitionEvent::updateCurrentAction(const BT::NodeStatus & status, const BT::TreeNode & node)
{
  if (status != BT::NodeStatus::SUCCESS) {
    current_action_ = node.name();
  }
}
}  // namespace behavior_tree_plugin
