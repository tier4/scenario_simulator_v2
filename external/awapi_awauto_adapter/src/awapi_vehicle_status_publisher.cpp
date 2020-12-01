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

#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <awapi_awauto_adapter/awapi_vehicle_status_publisher.hpp>
#include <quaternion_operation/quaternion_operation.h>

#include <string>
#include <limits>
using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;

namespace autoware_api
{
AutowareVehicleStatusPublisher::AutowareVehicleStatusPublisher(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("awapi_vehicle_status_publisher", options)
{
  // publisher
  pub_vehicle_status_ = this->create_publisher<VehicleStatus>("/awapi/vehicle/get/status", 1);
  sub_twist_ = create_subscription<TwistStamped>("/localization/twist",
      1, [&](const TwistStamped::SharedPtr msg_ptr) {twist_ptr_ = msg_ptr;});
}
void AutowareVehicleStatusPublisher::publish_vehicle_status()
{
  VehicleStatus vehicle_status = init_vehicle_status();
  vehicle_status.header.frame_id = "base_link";
  vehicle_status.header.stamp = get_clock()->now();
  pub_vehicle_status_->publish(vehicle_status);
  RCLCPP_INFO(this->get_logger(), " VehicleStatus %i",
    vehicle_status.header.stamp);
}
VehicleStatus AutowareVehicleStatusPublisher::init_vehicle_status()
{
  VehicleStatus status;
  // set default value
  if (std::numeric_limits<float>::has_quiet_NaN) {
    status.energy_level = std::numeric_limits<float>::quiet_NaN();
  }
  return status;
}
int32_t AutowareVehicleStatusPublisher::get_turn_signal_info()
{
  int turn_signal = 1;
  return turn_signal;
}
}  // namespace autoware_api
