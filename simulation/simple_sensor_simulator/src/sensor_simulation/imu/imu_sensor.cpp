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
  const auto applyNoise = [&](geometry_msgs::msg::Vector3 & v) {
    if (add_noise_) {
      v.x += noise_distribution_(random_generator_);
      v.y += noise_distribution_(random_generator_);
      v.z += noise_distribution_(random_generator_);
    }
  };

  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = current_ros_time;
  imu_msg.header.frame_id = "imu_frame";

  // Apply noise
  auto orientation_rpy = math::geometry::convertQuaternionToEulerAngle(status.pose.orientation);
  applyNoise(orientation_rpy);
  auto twist = status.action_status.twist;
  applyNoise(twist.angular);
  auto accel = status.action_status.accel;
  applyNoise(accel.linear);

  // Set data
  imu_msg.orientation = math::geometry::convertEulerAngleToQuaternion(orientation_rpy);
  imu_msg.angular_velocity.x = twist.angular.x;
  imu_msg.angular_velocity.y = twist.angular.y;
  imu_msg.angular_velocity.z = twist.angular.z;
  imu_msg.linear_acceleration.x = accel.linear.x;
  imu_msg.linear_acceleration.y = accel.linear.y;
  imu_msg.linear_acceleration.z = accel.linear.z;

  // Set covariance
  imu_msg.orientation_covariance = {std::pow(noise_standard_deviation_, 2), 0, 0, 0,
                                    std::pow(noise_standard_deviation_, 2), 0, 0, 0,
                                    std::pow(noise_standard_deviation_, 2)};
  imu_msg.angular_velocity_covariance = {std::pow(noise_standard_deviation_, 2), 0, 0, 0,
                                         std::pow(noise_standard_deviation_, 2), 0, 0, 0,
                                         std::pow(noise_standard_deviation_, 2)};
  imu_msg.linear_acceleration_covariance = {std::pow(noise_standard_deviation_, 2), 0, 0, 0,
                                            std::pow(noise_standard_deviation_, 2), 0, 0, 0,
                                            std::pow(noise_standard_deviation_, 2)};
  return imu_msg;
}
}  // namespace simple_sensor_simulator
