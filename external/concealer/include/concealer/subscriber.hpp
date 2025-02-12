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
#include <concealer/get_parameter.hpp>
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
      available<Message>() ? autoware.template create_subscription<Message>(
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
      available<Message>() ? autoware.template create_subscription<Message>(
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
      available<Message>() ? autoware.template create_subscription<Message>(
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
      available<Message>() ? autoware.template create_subscription<Message>(
                               topic, quality_of_service,
                               [this](const typename Message::ConstSharedPtr & message) {
                                 std::atomic_store(&current_value, message);
                               })
                           : nullptr)
  {
  }
};

template <typename... Messages>
struct Subscriber<std::tuple<Messages...>> : public Subscriber<Messages...>
{
  using type = std::tuple<Messages...>;

  using primary_type = std::tuple_element_t<0, type>;

  template <typename F, typename T, typename... Ts>
  constexpr auto any(F f, const Subscriber<T, Ts...> & x) -> bool
  {
    if constexpr (0 < sizeof...(Ts)) {
      return f(x) or any(f, static_cast<const Subscriber<Ts...> &>(x));
    } else {
      return f(x);
    }
  }

  template <typename... Ts>
  explicit Subscriber(const std::string & topic, Ts &&... xs)
  : Subscriber<Messages...>(topic, std::forward<decltype(xs)>(xs)...)
  {
    auto subscription_available = [](const auto & x) { return static_cast<bool>(x.subscription); };

    if (not any(subscription_available, static_cast<const Subscriber<Messages...> &>(*this))) {
      throw common::scenario_simulator_exception::Error(
        "No viable subscription for topic ", std::quoted(topic), " in ",
        getParameter<std::string>("architecture_type"), ".");
    }
  }
};
}  // namespace concealer

#endif  // CONCEALER__SUBSCRIBER_HPP_
