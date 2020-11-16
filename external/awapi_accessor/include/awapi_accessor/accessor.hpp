// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef AWAPI_ACCESSOR__ACCESSOR_HPP_
#define AWAPI_ACCESSOR__ACCESSOR_HPP_

#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <awapi_accessor/utility/visibility.h>
#include <rclcpp/rclcpp.hpp>

namespace autoware_api
{

#define DEFINE_SUBSCRIPTION(TYPE, NAME) \
  TYPE NAME; \
  rclcpp::Subscription<decltype(NAME)>::SharedPtr subscription_of_ ## NAME

class Accessor
  : public rclcpp::Node
{
  DEFINE_SUBSCRIPTION(autoware_api_msgs::msg::AwapiVehicleStatus, vehicle_get_status_);
  DEFINE_SUBSCRIPTION(autoware_api_msgs::msg::AwapiAutowareStatus, autoware_get_status_);

public:
  AWAPI_ACCESSOR_PUBLIC
  explicit Accessor(const rclcpp::NodeOptions &);
};

#undef DEFINE_SUBSCRIPTION

}  // namespace autoware_api

#endif  // AWAPI_ACCESSOR__ACCESSOR_HPP_
