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

#ifndef CONCEALER__SUBSCRIBER_WRAPPER_HPP_
#define CONCEALER__SUBSCRIBER_WRAPPER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace concealer
{
template <typename MessageType>
class SubscriberWrapper
{
private:
  typename MessageType::ConstSharedPtr current_value = std::make_shared<const MessageType>();
  typename rclcpp::Subscription<MessageType>::SharedPtr subscription;

public:
  auto operator()() const -> const MessageType & { return *current_value; }

  template <typename NodeInterface>
  SubscriberWrapper(
    std::string topic, NodeInterface & autoware_interface,
    std::function<void(const MessageType &)> callback = {})
  : subscription(autoware_interface.template create_subscription<MessageType>(
      topic, 1, [this, callback](const typename MessageType::ConstSharedPtr message) {
        current_value = message;
        if (current_value && callback) {
          callback(*current_value);
        }
      }))
  {
  }
};

template <typename MessageType>
class ThreadSafeSubscriberWrapper
{
private:
  typename MessageType::ConstSharedPtr current_value = std::make_shared<MessageType>();
  typename rclcpp::Subscription<MessageType>::SharedPtr subscription;

public:
  auto operator()() const -> MessageType { return *std::atomic_load(&current_value); }

  template <typename NodeInterface>
  ThreadSafeSubscriberWrapper(
    std::string topic, NodeInterface & autoware_interface,
    std::function<void(const MessageType &)> callback = {})
  : subscription(autoware_interface.template create_subscription<MessageType>(
      topic, 1, [this, callback](const typename MessageType::ConstSharedPtr message) {
        std::atomic_store(&current_value, message);
        if (current_value && callback) {
          callback(*std::atomic_load(&current_value));
        }
      }))
  {
  }
};
}  // namespace concealer

#endif  //CONCEALER__SUBSCRIBER_WRAPPER_HPP_
