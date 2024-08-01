// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <algorithm>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <simple_sensor_simulator/sensor_simulation/imu/imu_sensor.hpp>

namespace simple_sensor_simulator
{
template <>
auto ImuSensor<sensor_msgs::msg::Imu>::generateMessage(
  const rclcpp::Time & current_ros_time,
  const traffic_simulator_msgs::msg::EntityStatus & status) const -> const sensor_msgs::msg::Imu
{
  const auto applyNoise =
    [&](geometry_msgs::msg::Vector3 & v, std::normal_distribution<> & distribution) {
      v.x += distribution(random_generator_);
      v.y += distribution(random_generator_);
      v.z += distribution(random_generator_);
    };

  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = current_ros_time;
  imu_msg.header.frame_id = frame_id_;

  auto orientation_rpy = math::geometry::convertQuaternionToEulerAngle(status.pose.orientation);
  auto twist = status.action_status.twist;
  auto accel = status.action_status.accel;

  // Apply noise
  if (noise_standard_deviation_orientation_ > 0.0) {
    applyNoise(orientation_rpy, noise_distribution_orientation_);
  }
  if (noise_standard_deviation_twist_ > 0.0) {
    applyNoise(twist.angular, noise_distribution_twist_);
  }
  if (noise_standard_deviation_acceleration_ > 0.0) {
    applyNoise(accel.linear, noise_distribution_acceleration_);
  }

  // Apply gravity
  if (add_gravity_) {
    tf2::Quaternion orientation_quaternion;
    orientation_quaternion.setRPY(orientation_rpy.x, orientation_rpy.y, orientation_rpy.z);
    tf2::Matrix3x3 rotation_matrix(orientation_quaternion);
    tf2::Vector3 gravity(0.0, 0.0, -9.81);
    tf2::Vector3 transformed_gravity = rotation_matrix * gravity;
    accel.linear.x += transformed_gravity.x();
    accel.linear.y += transformed_gravity.y();
    accel.linear.z += transformed_gravity.z();
  }

  // Set data
  imu_msg.orientation = math::geometry::convertEulerAngleToQuaternion(orientation_rpy);
  imu_msg.angular_velocity.x = twist.angular.x;
  imu_msg.angular_velocity.y = twist.angular.y;
  imu_msg.angular_velocity.z = twist.angular.z;
  imu_msg.linear_acceleration.x = accel.linear.x;
  imu_msg.linear_acceleration.y = accel.linear.y;
  imu_msg.linear_acceleration.z = accel.linear.z;

  // Set covariance
  std::copy(
    orientation_covariance_.begin(), orientation_covariance_.end(),
    imu_msg.orientation_covariance.data());
  std::copy(
    angular_velocity_covariance_.begin(), angular_velocity_covariance_.end(),
    imu_msg.angular_velocity_covariance.data());
  std::copy(
    linear_acceleration_covariance_.begin(), linear_acceleration_covariance_.end(),
    imu_msg.linear_acceleration_covariance.data());
  return imu_msg;
}
}  // namespace simple_sensor_simulator
