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
#include <get_parameter/get_parameter.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <chrono>
#include <atomic>

namespace concealer
{
template <typename...>
struct Subscriber;

template <typename Message>
struct Subscriber<Message>
{
  typename Message::ConstSharedPtr current_value = std::make_shared<const Message>();

  typename rclcpp::Subscription<Message>::SharedPtr subscription;
  
  mutable std::atomic<std::size_t> message_count{0};
  mutable std::atomic<std::chrono::steady_clock::time_point> first_message_time{};

  auto operator()() const -> Message { return *std::atomic_load(&current_value); }

  template <typename Autoware, typename Callback>
  explicit Subscriber(
    const std::string & topic, const rclcpp::QoS & quality_of_service, Autoware & autoware,
    const Callback & callback)
  : subscription(
      available<Message>() ? autoware.template create_subscription<Message>(
                               topic, quality_of_service,
                               [this, callback, topic](const typename Message::ConstSharedPtr & message) {
                                 auto current_count = message_count.fetch_add(1) + 1;
                                 
                                 if (current_count <= 3 || (current_count % 50 == 0 && current_count<=150)) {
                                   auto now = std::chrono::steady_clock::now();
                                   
                                   auto expected_first_time = std::chrono::steady_clock::time_point{};
                                   if (current_count == 1) {
                                     first_message_time.compare_exchange_weak(expected_first_time, now);
                                   }
                                   
                                   double frequency = 0.0;
                                   if (current_count > 1) {
                                     auto first_time = first_message_time.load();
                                     if (first_time != std::chrono::steady_clock::time_point{}) {
                                       auto time_diff = std::chrono::duration<double>(now - first_time).count();
                                       frequency = (current_count - 1) / time_diff;
                                     }
                                   }
                                   
                                   RCLCPP_WARN(
                                     rclcpp::get_logger("DEBUG/concealer::Subscriber"),
                                     "Received message [no. %zu] on topic %s (freq: %.2f Hz)", current_count, topic.c_str(), frequency);
                                 }
                                 
                                 if (std::atomic_store(&current_value, message); current_value) {
                                   callback((*this)());
                                 }
                               })
                           : nullptr)
  {
    if (available<Message>()) {
      RCLCPP_WARN(
        rclcpp::get_logger("DEBUG/concealer::Subscriber"),
        "Created subscription for topic %s with callback", topic.c_str());
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("DEBUG/concealer::Subscriber"),
        "Message type not available for topic %s", topic.c_str());
    }
  }

  template <typename Autoware>
  explicit Subscriber(
    const std::string & topic, const rclcpp::QoS & quality_of_service, Autoware & autoware)
  : subscription(
      available<Message>() ? autoware.template create_subscription<Message>(
                               topic, quality_of_service,
                               [this, topic](const typename Message::ConstSharedPtr & message) {
                                 auto current_count = message_count.fetch_add(1) + 1;
                                 
                                 if (current_count <= 3 || (current_count % 50 == 0 && current_count<=150)) {
                                   auto now = std::chrono::steady_clock::now();
                                   
                                   auto expected_first_time = std::chrono::steady_clock::time_point{};
                                   if (current_count == 1) {
                                     first_message_time.compare_exchange_weak(expected_first_time, now);
                                   }
                                   
                                   double frequency = 0.0;
                                   if (current_count > 1) {
                                     auto first_time = first_message_time.load();
                                     if (first_time != std::chrono::steady_clock::time_point{}) {
                                       auto time_diff = std::chrono::duration<double>(now - first_time).count();
                                       frequency = (current_count - 1) / time_diff;
                                     }
                                   }
                                   
                                   RCLCPP_WARN(
                                     rclcpp::get_logger("DEBUG/concealer::Subscriber"),
                                     "Received message [no. %zu] on topic %s (freq: %.2f Hz)", current_count, topic.c_str(), frequency);
                                 }
                                 
                                 std::atomic_store(&current_value, message);
                               })
                           : nullptr)
  {
    if (available<Message>()) {
      RCLCPP_WARN(
        rclcpp::get_logger("DEBUG/concealer::Subscriber"),
        "Created subscription for topic %s", topic.c_str());
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("DEBUG/concealer::Subscriber"),
        "Message type not available for topic %s", topic.c_str());
    }
  }
};

template <typename Message, typename T, typename... Ts>
struct Subscriber<Message, T, Ts...> : public Subscriber<T, Ts...>
{
  typename Message::ConstSharedPtr current_value = nullptr;

  typename rclcpp::Subscription<Message>::SharedPtr subscription;
  
  mutable std::atomic<std::size_t> message_count{0};
  mutable std::atomic<std::chrono::steady_clock::time_point> first_message_time{};

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
                               [this, callback, topic](const typename Message::ConstSharedPtr & message) {
                                 auto current_count = message_count.fetch_add(1) + 1;
                                 
                                 if (current_count <= 3 || (current_count % 50 == 0 && current_count<=150)) {
                                   auto now = std::chrono::steady_clock::now();
                                   
                                   auto expected_first_time = std::chrono::steady_clock::time_point{};
                                   if (current_count == 1) {
                                     first_message_time.compare_exchange_weak(expected_first_time, now);
                                   }
                                   
                                   double frequency = 0.0;
                                   if (current_count > 1) {
                                     auto first_time = first_message_time.load();
                                     if (first_time != std::chrono::steady_clock::time_point{}) {
                                       auto time_diff = std::chrono::duration<double>(now - first_time).count();
                                       frequency = (current_count - 1) / time_diff;
                                     }
                                   }
                                   
                                   RCLCPP_WARN(
                                     rclcpp::get_logger("DEBUG/concealer::Subscriber"),
                                     "Received message [no. %zu] on topic %s (multi-type subscriber) (freq: %.2f Hz)", current_count, topic.c_str(), frequency);
                                 }
                                 
                                 if (std::atomic_store(&current_value, message); current_value) {
                                   callback((*this)());
                                 }
                               })
                           : nullptr)
  {
    if (available<Message>()) {
      RCLCPP_WARN(
        rclcpp::get_logger("DEBUG/concealer::Subscriber"),
        "Created multi-type subscription for topic %s with callback", topic.c_str());
    }
  }

  template <typename Autoware>
  explicit Subscriber(
    const std::string & topic, const rclcpp::QoS & quality_of_service, Autoware & autoware)
  : Subscriber<T, Ts...>(topic, quality_of_service, autoware),
    subscription(
      available<Message>() ? autoware.template create_subscription<Message>(
                               topic, quality_of_service,
                               [this, topic](const typename Message::ConstSharedPtr & message) {
                                 auto current_count = message_count.fetch_add(1) + 1;
                                 
                                 if (current_count <= 3 || (current_count % 50 == 0 && current_count<=150)) {
                                   auto now = std::chrono::steady_clock::now();
                                   
                                   auto expected_first_time = std::chrono::steady_clock::time_point{};
                                   if (current_count == 1) {
                                     first_message_time.compare_exchange_weak(expected_first_time, now);
                                   }
                                   
                                   double frequency = 0.0;
                                   if (current_count > 1) {
                                     auto first_time = first_message_time.load();
                                     if (first_time != std::chrono::steady_clock::time_point{}) {
                                       auto time_diff = std::chrono::duration<double>(now - first_time).count();
                                       frequency = (current_count - 1) / time_diff;
                                     }
                                   }
                                   
                                   RCLCPP_WARN(
                                     rclcpp::get_logger("DEBUG/concealer::Subscriber"),
                                     "Received message [no. %zu] on topic %s (multi-type subscriber) (freq: %.2f Hz)", current_count, topic.c_str(), frequency);
                                 }
                                 
                                 std::atomic_store(&current_value, message);
                               })
                           : nullptr)
  {
    if (available<Message>()) {
      RCLCPP_WARN(
        rclcpp::get_logger("DEBUG/concealer::Subscriber"),
        "Created multi-type subscription for topic %s", topic.c_str());
    }
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
      RCLCPP_WARN(
        rclcpp::get_logger("DEBUG/concealer::Subscriber"),
        "No viable subscription for topic %s in %s", topic.c_str(),
        common::getParameter<std::string>("architecture_type").c_str());
      throw common::scenario_simulator_exception::Error(
        "No viable subscription for topic ", std::quoted(topic), " in ",
        common::getParameter<std::string>("architecture_type"), ".");
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("DEBUG/concealer::Subscriber"),
        "Created tuple subscriber for topic %s", topic.c_str());
    }
  }
};
}  // namespace concealer

#endif  // CONCEALER__SUBSCRIBER_HPP_
