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

#include <utility>

#define DEFINE_SUBSCRIPTION(TYPE) \
private: \
  TYPE current_value_of_ ## TYPE; \
  rclcpp::Subscription<TYPE>::SharedPtr subscription_of_ ## TYPE; \
public: \
  const auto & get ## TYPE() const noexcept \
  { \
    return current_value_of_ ## TYPE; \
  } \
  static_assert(true, "")

#define DEFINE_PUBLISHER(TYPE) \
private: \
  rclcpp::Publisher<TYPE>::SharedPtr publisher_of_ ## TYPE; \
public: \
  template \
  < \
    typename ... Ts \
  > \
  decltype(auto) set ## TYPE(Ts && ... xs) const \
  { \
    return (*publisher_of_ ## TYPE).publish(std::forward<decltype(xs)>(xs)...); \
  } \
  static_assert(true, "")

#define INIT_SUBSCRIPTION(TYPE, TOPIC) \
  subscription_of_ ## TYPE( \
    (*node).template create_subscription<TYPE>( \
      TOPIC, 1, \
      [this](const TYPE::SharedPtr message) \
      { \
        current_value_of_ ## TYPE = *message; \
      }))

#define INIT_PUBLISHER(TYPE, TOPIC) \
  publisher_of_ ## TYPE( \
    (*node).template create_publisher<TYPE>(TOPIC, 10))

#endif  // AWAPI_ACCESSOR__DEFINE_MACRO_HPP_
