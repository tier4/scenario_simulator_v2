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

#include <rclcpp/create_subscription.hpp>
#include <rclcpp/qos.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/monitor/linear_jerk_subscription_value_policy.hpp>

namespace traffic_simulator::entity
{
LinearJerkSubscriptionValuePolicy::LinearJerkSubscriptionValuePolicy(
  NodeTopicsInterfacePtr node_topics_interface_ptr, const std::string & topic_name)
: subscription_ptr_(rclcpp::create_subscription<MessageType>(
    node_topics_interface_ptr, topic_name, rclcpp::SensorDataQoS(),
    [this](const MessageType::ConstSharedPtr msg) { linear_jerk_ = msg->data; }))
{
}

auto LinearJerkSubscriptionValuePolicy::getValue(const EntityBase &) const -> double
{
  return linear_jerk_;
}
}  // namespace traffic_simulator::entity
