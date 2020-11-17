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

#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <awapi_accessor/utility/visibility.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <utility>

namespace autoware_api
{

#define DEFINE_SUBSCRIPTION(TYPE) \
protected: \
  TYPE current_value_of_ ## TYPE; \
 \
private: \
  rclcpp::Subscription<TYPE>::SharedPtr subscription_of_ ## TYPE; \
 \
public: \
  const auto & get ## TYPE() const noexcept \
  { \
    return current_value_of_ ## TYPE; \
  } \
  static_assert(true, "")


#define DEFINE_PUBLICATION(TYPE) \
private: \
  rclcpp::Publisher<TYPE>::SharedPtr publisher_of_ ## TYPE; \
 \
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


class Accessor
  : public rclcpp::Node
{
  /** ---- AutowareEngage ------------------------------------------------------
   *
   *  Topic: /awapi/autoware/put/engage
   *
   * ------------------------------------------------------------------------ */
  using AutowareEngage = std_msgs::msg::Bool;

  DEFINE_PUBLICATION(AutowareEngage);

  /** ---- VehicleVelocity -----------------------------------------------------
   *
   *  Set upper bound of velocity.
   *
   *  Topic: /awapi/vehicle/put/velocity
   *
   * ------------------------------------------------------------------------ */
  using VehicleVelocity = std_msgs::msg::Float32;

  DEFINE_PUBLICATION(VehicleVelocity);

  /** ---- AutowareStatus ------------------------------------------------------
   *
   *  Topic: /awapi/autoware/get/status
   *
   * ------------------------------------------------------------------------ */
  using AutowareStatus = autoware_api_msgs::msg::AwapiAutowareStatus;

  DEFINE_SUBSCRIPTION(AutowareStatus);

  /** ---- VehicleStatus -------------------------------------------------------
   *
   *  Topic: /awapi/vehicle/get/status
   *
   * ------------------------------------------------------------------------ */
  using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;

  DEFINE_SUBSCRIPTION(VehicleStatus);

public:
  AWAPI_ACCESSOR_PUBLIC
  explicit Accessor(const rclcpp::NodeOptions &);
};

#undef DEFINE_SUBSCRIPTION
#undef DEFINE_PUBLICATION

}  // namespace autoware_api

#endif  // AWAPI_ACCESSOR__ACCESSOR_HPP_
