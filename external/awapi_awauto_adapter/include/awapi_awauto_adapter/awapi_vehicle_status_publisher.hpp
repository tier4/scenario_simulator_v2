// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#ifndef AWAPI_AWAUTO_ADAPTER__AWAPI_VEHICLE_STATUS_PUBLISHER_HPP_
#define AWAPI_AWAUTO_ADAPTER__AWAPI_VEHICLE_STATUS_PUBLISHER_HPP_
#include <awapi_awauto_adapter/utility/visibility.h>
#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

namespace autoware_api
{
class AutowareVehicleStatusPublisher : public rclcpp::Node
{
  /**
   *  VehicleStatus Topic: /awapi/vehicle/get/status
   */
  using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  rclcpp::Publisher<VehicleStatus>::SharedPtr pub_vehicle_status_;
  VehicleStatus vehicle_status_;
  rclcpp::TimerBase::SharedPtr timer_vehicle_status_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  TwistStamped::SharedPtr twist_ptr_;
  /**
   * @brief initialize vehicle status
   * @return VehicleStatus
   */
  VehicleStatus init_vehicle_status();
  /**
   * @brief get turn signal info
   * @return int32_t
   * @todo make turn signal info
   */
  int32_t get_turn_signal_info();

public:
  AWAPI_AWAUTO_ADAPTER_PUBLIC
  explicit AutowareVehicleStatusPublisher(const rclcpp::NodeOptions &);
  /**
   * @brief publish vehicle status
   * @return none
   */
  void publish_vehicle_status();
};
}  // namespace autoware_api
#endif  // AWAPI_AWAUTO_ADAPTER__AWAPI_VEHICLE_STATUS_PUBLISHER_HPP_
