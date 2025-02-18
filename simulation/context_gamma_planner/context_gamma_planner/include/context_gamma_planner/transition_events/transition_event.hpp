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

#ifndef context_gamma_planner__TRANSITION_EVENTS__TRANSITION_EVENT_HPP_
#define context_gamma_planner__TRANSITION_EVENTS__TRANSITION_EVENT_HPP_

#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace context_gamma_planner
{
class TransitionEvent
{
public:
  TransitionEvent(BT::TreeNode * root_node);

protected:
  virtual void callback(
    BT::Duration timestamp, const BT::TreeNode & node, BT::NodeStatus prev_status,
    BT::NodeStatus status) = 0;
  void updateCurrentAction(const BT::NodeStatus & status, const BT::TreeNode & node);
  //std::shared_ptr<BT::TreeNode> root_node_;
  BT::TimePoint first_timestamp_;
  std::vector<BT::TreeNode::StatusChangeSubscriber> subscribers_;
  BT::TimestampType type_;
  std::string current_action_;
};
}  // namespace context_gamma_planner

#endif  // context_gamma_planner__TRANSITION_EVENTS__TRANSITION_EVENT_HPP_
