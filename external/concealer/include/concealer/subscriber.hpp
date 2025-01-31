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

#ifndef CONCEALER__SUBSCRIBER_HPP_
#define CONCEALER__SUBSCRIBER_HPP_

#include <concealer/convert.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace concealer
{
template <typename...>
struct Subscriber;

template <typename Message>
struct Subscriber<Message>
{
  typename Message::ConstSharedPtr current_value = std::make_shared<const Message>();

  typename rclcpp::Subscription<Message>::SharedPtr subscription;

  auto operator()() const -> Message { return *std::atomic_load(&current_value); }

  template <typename Autoware, typename Callback>
  explicit Subscriber(
    const std::string & topic, const rclcpp::QoS & quality_of_service, Autoware & autoware,
    const Callback & callback)
  : subscription(
      available<Message>(autoware.get_node_parameters_interface())
        ? autoware.template create_subscription<Message>(
            topic, quality_of_service,
            [this, callback](const typename Message::ConstSharedPtr & message) {
              if (std::atomic_store(&current_value, message); current_value) {
                callback((*this)());
              }
            })
        : nullptr)
  {
  }

  template <typename Autoware>
  explicit Subscriber(
    const std::string & topic, const rclcpp::QoS & quality_of_service, Autoware & autoware)
  : subscription(
      available<Message>(autoware.get_node_parameters_interface())
        ? autoware.template create_subscription<Message>(
            topic, quality_of_service,
            [this](const typename Message::ConstSharedPtr & message) {
              std::atomic_store(&current_value, message);
            })
        : nullptr)
  {
  }
};

template <typename Message, typename T, typename... Ts>
struct Subscriber<Message, T, Ts...> : public Subscriber<T, Ts...>
{
  typename Message::ConstSharedPtr current_value = nullptr;

  typename rclcpp::Subscription<Message>::SharedPtr subscription;

  auto operator()() const -> Message
  {
    if (auto value = std::atomic_load(&current_value)) {
      return *value;
    } else {
      return convert<Message>(Subscriber<T, Ts...>::operator()());
    }
  }

  template <typename Autoware, typename Callback>
  explicit Subscriber(
    const std::string & topic, const rclcpp::QoS & quality_of_service, Autoware & autoware,
    const Callback & callback)
  : Subscriber<T, Ts...>(topic, quality_of_service, autoware, callback),
    subscription(
      available<Message>(autoware.get_node_parameters_interface())
        ? autoware.template create_subscription<Message>(
            topic, quality_of_service,
            [this, callback](const typename Message::ConstSharedPtr & message) {
              if (std::atomic_store(&current_value, message); current_value) {
                callback((*this)());
              }
            })
        : nullptr)
  {
  }

  template <typename Autoware>
  explicit Subscriber(
    const std::string & topic, const rclcpp::QoS & quality_of_service, Autoware & autoware)
  : Subscriber<T, Ts...>(topic, quality_of_service, autoware),
    subscription(
      available<Message>(autoware.get_node_parameters_interface())
        ? autoware.template create_subscription<Message>(
            topic, quality_of_service,
            [this](const typename Message::ConstSharedPtr & message) {
              std::atomic_store(&current_value, message);
            })
        : nullptr)
  {
  }
};

template <typename... Ts>
struct Subscriber<std::tuple<Ts...>> : public Subscriber<Ts...>
{
  using Subscriber<Ts...>::Subscriber;
};
}  // namespace concealer

#endif  // CONCEALER__SUBSCRIBER_HPP_
