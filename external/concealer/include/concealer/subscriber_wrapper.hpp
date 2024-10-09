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
enum class ThreadSafety : bool { unsafe, safe };

template <typename MessageType, ThreadSafety thread_safety = ThreadSafety::unsafe>
class SubscriberWrapper
{
  typename MessageType::ConstSharedPtr current_value = std::make_shared<const MessageType>();

  typename rclcpp::Subscription<MessageType>::SharedPtr subscription;

public:
  auto operator()() const -> decltype(auto)
  {
    if constexpr (thread_safety == ThreadSafety::unsafe) {
      return *current_value;
    } else {
      return *std::atomic_load(&current_value);
    }
  }

  template <typename NodeInterface>
  SubscriberWrapper(
    const std::string & topic, const rclcpp::QoS & quality_of_service,
    NodeInterface & autoware_interface,
    const std::function<void(const MessageType &)> & callback = {})
  : subscription(autoware_interface.template create_subscription<MessageType>(
      topic, quality_of_service,
      [this, callback](const typename MessageType::ConstSharedPtr message) {
        if constexpr (thread_safety == ThreadSafety::safe) {
          std::atomic_store(&current_value, message);
          if (current_value and callback) {
            callback(*std::atomic_load(&current_value));
          }
        } else {
          if (current_value = message; current_value and callback) {
            callback(*current_value);
          }
        }
      }))
  {
  }
};
}  // namespace concealer

#endif  // CONCEALER__SUBSCRIBER_WRAPPER_HPP_
