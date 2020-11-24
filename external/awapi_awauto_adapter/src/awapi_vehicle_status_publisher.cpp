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
: rclcpp::Node("awapi_vehicle_status_publisher", options)
{
  // publisher
  pub_vehicle_status_ = this->create_publisher<VehicleStatus>("/awapi/vehicle/get/status", 1);
}
void AutowareVehicleStatusPublisher::publish_vehicle_status()
{
  VehicleStatus vehicle_status = init_vehicle_status();
  vehicle_status.header.frame_id = "base_link";
  vehicle_status.header.stamp = get_clock()->now();
  get_pose_info(&vehicle_status);
  get_steer_info(&vehicle_status);
  get_steer_info(&vehicle_status);
  get_vehicle_cmd_info(&vehicle_status);
  get_twist_info(&vehicle_status);
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

  // convert quaternion to euler
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  status->eulerangle.yaw = yaw;
  status->eulerangle.pitch = pitch;
  status->eulerangle.roll = roll;
}
void AutowareVehicleStatusPublisher::get_steer_info(VehicleStatus * status)
{
  const double steering = 0.2;
  status->steering = steering;
  const double steer_vel = 0.1;
  status->steering_velocity = steer_vel;
}
void AutowareVehicleStatusPublisher::get_vehicle_cmd_info(
  VehicleStatus * status)
{
  const double acceleration = 0.1;
  const double velocity = 0.2;
  const double steering_angle = 0.3;
  const double steering_angle_velocity = 0.4;
  status->target_acceleration = acceleration;
  status->target_velocity = velocity;
  status->target_steering = steering_angle;
  status->target_steering_velocity = steering_angle_velocity;
}
void AutowareVehicleStatusPublisher::get_turn_signal_info(VehicleStatus * status)
{
  int turn_signal = 1;
  // get turn signal
  status->turn_signal = turn_signal;
}
void AutowareVehicleStatusPublisher::get_twist_info(VehicleStatus * status)
{
  geometry_msgs::msg::TwistStamped twist_gen;
  twist_gen.twist.linear.x = 1.0;
  status->velocity = twist_gen.twist.linear.x;
  status->angular_velocity = twist_gen.twist.angular.z;
}
}  // namespace autoware_api
