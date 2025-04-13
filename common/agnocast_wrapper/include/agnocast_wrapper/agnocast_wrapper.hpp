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

#ifndef AGNOCAST_WRAPPER__AGNOCAST_WRAPPER_HPP_
#define AGNOCAST_WRAPPER__AGNOCAST_WRAPPER_HPP_

#include <string>
#include <utility>

/// @sa https://github.com/veqcc/autoware.universe/blob/252ae60788a8388585780a6b1935a5682c688464/common/autoware_agnocast_wrapper/include/autoware_agnocast_wrapper/autoware_agnocast_wrapper.hpp

#ifdef USE_AGNOCAST_ENABLED
#pragma message("Building with USE_AGNOCAST_ENABLED defined")
#else
#pragma message("Building without USE_AGNOCAST_ENABLED defined")
#endif

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast/agnocast.hpp>
#else
#include <memory>
#include <rclcpp/rclcpp.hpp>
#endif  // USE_AGNOCAST_ENABLED

namespace agnocast_wrapper
{
/**
 * @brief Templated types
 */
#ifdef USE_AGNOCAST_ENABLED
template <typename MessageT>
using MessagePtr = agnocast::ipc_shared_ptr<MessageT>;

template <typename MessageT>
using SubscriptionPtr = typename agnocast::Subscription<MessageT>::SharedPtr;

template <typename MessageT>
using PublisherPtr = typename agnocast::Publisher<MessageT>::SharedPtr;

using SubscriptionOptions = agnocast::SubscriptionOptions;
using PublisherOptions = agnocast::PublisherOptions;
#else
template <typename MessageT>
using MessagePtr = std::shared_ptr<MessageT>;

template <typename MessageT>
using SubscriptionPtr = typename rclcpp::Subscription<MessageT>::SharedPtr;

template <typename MessageT>
using PublisherPtr = typename rclcpp::Publisher<MessageT>::SharedPtr;

using SubscriptionOptions = rclcpp::SubscriptionOptions;
using PublisherOptions = rclcpp::PublisherOptions;
#endif  // USE_AGNOCAST_ENABLED

/**
 * @brief Create subscription
 */
template <typename MessageT, typename CallbackT, typename NodeT>
auto create_subscription(
  NodeT & node, const std::string & topic_name, const rclcpp::QoS & qos, CallbackT && callback,
  const SubscriptionOptions & options = SubscriptionOptions{}) -> SubscriptionPtr<MessageT>
{
#ifdef USE_AGNOCAST_ENABLED
  if constexpr (std::is_same_v<std::decay_t<NodeT>, rclcpp::Node::SharedPtr>) {
    return agnocast::create_subscription<MessageT>(
      node.get(), topic_name, qos, std::forward<CallbackT>(callback), options);
  } else {
    return agnocast::create_subscription<MessageT>(
      node, topic_name, qos, std::forward<CallbackT>(callback), options);
  }
#else
  if constexpr (std::is_same_v<std::decay_t<NodeT>, rclcpp::Node::SharedPtr>) {
    return node->template create_subscription<MessageT>(
      topic_name, qos, std::forward<CallbackT>(callback), options);
  } else {
    return node.template create_subscription<MessageT>(
      topic_name, qos, std::forward<CallbackT>(callback), options);
  }
#endif  // USE_AGNOCAST_ENABLED
}

/**
 * @brief Create publisher
 */
template <typename MessageT, typename NodeT>
auto create_publisher(
  NodeT & node, const std::string & topic_name, const rclcpp::QoS & qos,
  const PublisherOptions & options = PublisherOptions{}) -> PublisherPtr<MessageT>
{
#ifdef USE_AGNOCAST_ENABLED
  if constexpr (std::is_same_v<std::decay_t<NodeT>, rclcpp::Node::SharedPtr>) {
    return agnocast::create_publisher<MessageT>(node.get(), topic_name, qos, options);
  } else {
    return agnocast::create_publisher<MessageT>(&node, topic_name, qos, options);
  }
#else
  if constexpr (std::is_same_v<std::decay_t<NodeT>, rclcpp::Node::SharedPtr>) {
    return node->template create_publisher<MessageT>(topic_name, qos, options);
  } else {
    return node.template create_publisher<MessageT>(topic_name, qos, options);
  }
#endif  // USE_AGNOCAST_ENABLED
}

/**
 * @brief Create message object pointer
 * @param publisher_ptr pointer to the publisher for which the message will be
 * created. The message type will be deduced from the passed publisher.
 * @return pointer to the message - dependent on the publisher type
 */
template <typename PublisherPtrT>
auto create_message(PublisherPtrT publisher_ptr)
{
#ifdef USE_AGNOCAST_ENABLED
  return publisher_ptr->borrow_loaned_message();
#else
  using ROSMessageT =
    typename std::remove_reference<decltype(*publisher_ptr)>::type::ROSMessageType;
  return std::make_unique<ROSMessageT>();
#endif  // USE_AGNOCAST_ENABLED
}
}  // namespace agnocast_wrapper

#endif  // AGNOCAST_WRAPPER__AGNOCAST_WRAPPER_HPP_
