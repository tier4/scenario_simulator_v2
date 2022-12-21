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

#ifndef TRAFFIC_SIMULATOR__ENTITY__MONITOR__LINEAR_JERK_SUBSCRIPTION_VALUE_POLICY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__MONITOR__LINEAR_JERK_SUBSCRIPTION_VALUE_POLICY_HPP_

#include <memory>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/subscription.hpp>
#include <string>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <traffic_simulator/entity/entity_base.hpp>

namespace traffic_simulator::entity
{
class LinearJerkSubscriptionValuePolicy
{
public:
  using MessageType = tier4_debug_msgs::msg::Float32Stamped;
  using SubscriptionPtr = std::shared_ptr<rclcpp::Subscription<MessageType>>;
  using NodeTopicsInterfacePtr = std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>;

  LinearJerkSubscriptionValuePolicy(
    NodeTopicsInterfacePtr node_topics_interface_ptr, const std::string & topic_name);

  auto getValue(EntityBase & entity) const -> double;

private:
  SubscriptionPtr subscription_ptr_;
  double linear_jerk_ = 0;
};
}  // namespace traffic_simulator::entity

#endif  // TRAFFIC_SIMULATOR__ENTITY__MONITOR__LINEAR_JERK_SUBSCRIPTION_VALUE_POLICY_HPP_
