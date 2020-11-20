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

#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <awapi_awauto_adapter/awapi_vehicle_status_publisher.hpp>

#include <string>
#include <limits>
using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;

namespace autoware_api
{
AutowareVehicleStatusPublisher::AutowareVehicleStatusPublisher(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("awapi_vehicle_state_publisher", options)
{
  // publisher
  pub_vehicle_status_ = this->create_publisher<VehicleStatus>("/awapi/vehicle/get/status", 1);
}
void AutowareVehicleStatusPublisher::publish_vehicle_status()
{
  VehicleStatus vehicle_status = init_vehicle_status();
  vehicle_status.header.frame_id = "base_link";
  vehicle_status.header.stamp = get_clock()->now();
  vehicle_status.velocity = 0.1;
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
void AutowareVehicleStatusPublisher::get_pose_info(VehicleStatus * status)
{
  geometry_msgs::msg::Pose pose;
  status->pose = pose;
  tf2::Quaternion q(
    status->pose.orientation.x,
    status->pose.orientation.y,
    status->pose.orientation.z,
    status->pose.orientation.w);
  tf2::getEulerYPR(pose_ptr->pose.orientation, yaw, pitch, roll);

  // convert quaternion to euler
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  status->eulerangle.yaw = yaw;
  status->eulerangle.pitch = pitch;
  status->eulerangle.roll = roll;
}

}  // namespace autoware_api
