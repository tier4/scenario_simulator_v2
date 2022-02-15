// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef CONCEALER__DEFINE_MACRO_HPP_
#define CONCEALER__DEFINE_MACRO_HPP_

#include <mutex>
#include <utility>

#define CONCEALER_CURRENT_VALUE_OF(TYPE) current_value_of_##TYPE

#define CONCEALER_DEFINE_CLIENT(TYPE)                                                            \
private:                                                                                         \
  rclcpp::Client<TYPE>::SharedPtr client_of_##TYPE;                                              \
                                                                                                 \
public:                                                                                          \
  auto request##TYPE(const TYPE::Request::SharedPtr & request)->void                             \
  {                                                                                              \
    if (not client_of_##TYPE->service_is_ready()) {                                              \
      RCLCPP_INFO_STREAM(                                                                        \
        static_cast<Autoware &>(*this).get_logger(), #TYPE " service is not ready.");            \
    } else {                                                                                     \
      auto future = client_of_##TYPE->async_send_request(request);                               \
      if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {               \
        RCLCPP_INFO_STREAM(                                                                      \
          static_cast<Autoware &>(*this).get_logger(), #TYPE " service request has timed out."); \
      } else {                                                                                   \
        RCLCPP_INFO_STREAM(                                                                      \
          static_cast<Autoware &>(*this).get_logger(),                                           \
          #TYPE " service request has been accepted"                                             \
            << (future.get()->status.message.empty()                                             \
                  ? "."                                                                          \
                  : " (" + future.get()->status.message + ")."));                                \
      }                                                                                          \
    }                                                                                            \
  }                                                                                              \
  static_assert(true, "")

#define DEFINE_SUBSCRIPTION(TYPE)                                  \
private:                                                           \
  TYPE CONCEALER_CURRENT_VALUE_OF(TYPE);                           \
  rclcpp::Subscription<TYPE>::SharedPtr subscription_of_##TYPE;    \
                                                                   \
public:                                                            \
  auto get##TYPE() const->const auto &                             \
  {                                                                \
    const auto lock = static_cast<const Autoware &>(*this).lock(); \
    return CONCEALER_CURRENT_VALUE_OF(TYPE);                       \
  }                                                                \
  static_assert(true, "")

#define DEFINE_PUBLISHER(TYPE)                                       \
private:                                                             \
  rclcpp::Publisher<TYPE>::SharedPtr publisher_of_##TYPE;            \
                                                                     \
public:                                                              \
  auto set##TYPE(const TYPE & message)->decltype(auto)               \
  {                                                                  \
    return std::atomic_load(&publisher_of_##TYPE)->publish(message); \
  }                                                                  \
  static_assert(true, "")

#define CONCEALER_INIT_CLIENT(TYPE, SERVICE_NAME)                               \
  client_of_##TYPE(static_cast<Autoware &>(*this).template create_client<TYPE>( \
    SERVICE_NAME, rmw_qos_profile_default))

#define INIT_SUBSCRIPTION(TYPE, TOPIC, ERROR_CHECK)                                         \
  subscription_of_##TYPE(static_cast<Autoware &>(*this).template create_subscription<TYPE>( \
    TOPIC, 1, [this](const TYPE::SharedPtr message) {                                       \
      const auto lock = static_cast<Autoware &>(*this).lock();                              \
      CONCEALER_CURRENT_VALUE_OF(TYPE) = *message;                                          \
      ERROR_CHECK();                                                                        \
    }))

#define INIT_PUBLISHER(TYPE, TOPIC) \
  publisher_of_##TYPE(              \
    static_cast<Node &>(*this).template create_publisher<TYPE>(TOPIC, rclcpp::QoS(1).reliable()))

#endif  // CONCEALER__DEFINE_MACRO_HPP_
