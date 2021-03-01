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

#ifndef AWAPI_ACCESSOR__DEFINE_MACRO_HPP_
#define AWAPI_ACCESSOR__DEFINE_MACRO_HPP_

#include <mutex>
#include <utility>

#define CURRENT_VALUE_OF(TYPE) current_value_of_ ## TYPE

#define DEFINE_SUBSCRIPTION(TYPE) \
private: \
  TYPE CURRENT_VALUE_OF(TYPE); \
  rclcpp::Subscription<TYPE>::SharedPtr subscription_of_ ## TYPE; \
public: \
  const auto & get ## TYPE() \
  { \
    auto lock = std::unique_lock<decltype(mutex)>(mutex); \
    return CURRENT_VALUE_OF(TYPE); \
  } \
  static_assert(true, "")

#define DEFINE_PUBLISHER(TYPE) \
private: \
  rclcpp::Publisher<TYPE>::SharedPtr publisher_of_ ## TYPE; \
public: \
  decltype(auto) set ## TYPE(const TYPE & message) \
  { \
    return std::atomic_load(&publisher_of_ ## TYPE)->publish(message); \
  } \
  static_assert(true, "")

#define INIT_SUBSCRIPTION(TYPE, TOPIC, ERROR_CHECK) \
  subscription_of_ ## TYPE( \
    create_subscription<TYPE>( \
      TOPIC, 1, \
      [this](const TYPE::SharedPtr message) \
      { \
        auto lock = std::unique_lock<decltype(mutex)>(mutex); \
        CURRENT_VALUE_OF(TYPE) = *message; \
        ERROR_CHECK(); \
      }))

#define INIT_PUBLISHER(TYPE, TOPIC) \
  publisher_of_ ## TYPE( \
    create_publisher<TYPE>( \
      TOPIC, \
      rclcpp::QoS(1).reliable().transient_local()))

#endif  // AWAPI_ACCESSOR__DEFINE_MACRO_HPP_
