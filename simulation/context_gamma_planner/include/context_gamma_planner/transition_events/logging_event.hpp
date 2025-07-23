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

#ifndef CONTEXT_GAMMA_PLANNER__TRANSITION_EVENTS__LOGGING_EVENT_HPP_
#define CONTEXT_GAMMA_PLANNER__TRANSITION_EVENTS__LOGGING_EVENT_HPP_

#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "context_gamma_planner/transition_events/transition_event.hpp"

namespace context_gamma_planner
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
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__TRANSITION_EVENTS__LOGGING_EVENT_HPP_
