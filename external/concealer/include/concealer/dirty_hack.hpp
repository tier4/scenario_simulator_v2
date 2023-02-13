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

#ifndef CONCEALER__DIRTY_HACK_HPP_
#define CONCEALER__DIRTY_HACK_HPP_

#include <utility>

#define CONCEALER_DEFINE_SUBSCRIPTION(TYPE, ...)                                 \
private:                                                                         \
  TYPE::ConstSharedPtr current_value_of_##TYPE = std::make_shared<const TYPE>(); \
  rclcpp::Subscription<TYPE>::SharedPtr subscription_of_##TYPE;                  \
                                                                                 \
public:                                                                          \
  auto get##TYPE() const->TYPE __VA_ARGS__ { return *current_value_of_##TYPE; }  \
  static_assert(true, "")

#define CONCEALER_INIT_SUBSCRIPTION(TYPE, TOPIC)                                                \
  subscription_of_##TYPE(static_cast<AutowareUser &>(*this).template create_subscription<TYPE>( \
    TOPIC, 1, [this](const TYPE::ConstSharedPtr message) { current_value_of_##TYPE = message; }))

#endif  // CONCEALER__DIRTY_HACK_HPP_
