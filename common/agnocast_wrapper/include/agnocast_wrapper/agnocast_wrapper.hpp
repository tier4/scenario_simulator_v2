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

/**
 * @brief Define includes
 */
#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/agnocast.hpp>

#else

#include <memory>
#include <rclcpp/rclcpp.hpp>

#endif  // USE_AGNOCAST_ENABLED

namespace agnocast_wrapper
{
/**
 * @brief Define types for agnocast_wrapper
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
 * @brief Define wrapper functions
 */

// clang-format off
template<
  typename MessageT,
  typename CallbackT,
  typename NodePointerT>
SubscriptionPtr<MessageT>
create_subscription(
  NodePointerT node_ptr,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  CallbackT && callback,
  const SubscriptionOptions & options = SubscriptionOptions{})
// clang-format on
{
#ifdef USE_AGNOCAST_ENABLED
  if constexpr (std::is_same_v<std::decay_t<NodePointerT>, rclcpp::Node::SharedPtr>) {
    return agnocast::create_subscription<MessageT>(
      node_ptr.get(), topic_name, qos, std::forward<CallbackT>(callback), options);
  } else {
    return agnocast::create_subscription<MessageT>(
      node_ptr, topic_name, qos, std::forward<CallbackT>(callback), options);
  }
#else
  return node_ptr->template create_subscription<MessageT>(
    topic_name, qos, std::forward<CallbackT>(callback), options);
#endif  // USE_AGNOCAST_ENABLED
}

// clang-format off
template<
  typename MessageT,
  typename NodePointerT>
PublisherPtr<MessageT>
create_publisher(
  NodePointerT node_ptr,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const PublisherOptions & options = PublisherOptions{})
// clang-format on
{
#ifdef USE_AGNOCAST_ENABLED
  if constexpr (std::is_same_v<std::decay_t<NodePointerT>, rclcpp::Node::SharedPtr>) {
    return agnocast::create_publisher<MessageT>(node_ptr.get(), topic_name, qos, options);
  } else {
    return agnocast::create_publisher<MessageT>(node_ptr, topic_name, qos, options);
  }
#else
  return node_ptr->template create_publisher<MessageT>(topic_name, qos, options);
#endif  // USE_AGNOCAST_ENABLED
}

/**
 * @brief Create a message object pointer
 * @param publisher_ptr pointer to the publisher for which the message will be created. The message type will be deduced from the passed publisher.
 * @return pointer to the message, the pointer type will be different if agnocast is enabled
 */
template <typename PublisherPtrT>
auto create_message(PublisherPtrT publisher_ptr)
{
#ifdef USE_AGNOCAST_ENABLED
  return publisher_ptr->borrow_loaned_message();
#else
  /// @sa https://github.com/veqcc/autoware.universe/blob/252ae60788a8388585780a6b1935a5682c688464/common/autoware_agnocast_wrapper/include/autoware_agnocast_wrapper/autoware_agnocast_wrapper.hpp#L54C44-L54C138
  using ROSMessageT =
    typename std::remove_reference<decltype(*publisher_ptr)>::type::ROSMessageType;
  return std::make_unique<ROSMessageT>();
#endif  // USE_AGNOCAST_ENABLED
}
}  // namespace agnocast_wrapper

#endif  // AGNOCAST_WRAPPER__AGNOCAST_WRAPPER_HPP_
