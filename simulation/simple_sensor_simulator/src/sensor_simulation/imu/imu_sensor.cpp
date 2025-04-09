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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

// #include <Eigen/Geometry>
#include <algorithm>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <simple_sensor_simulator/sensor_simulation/imu/imu_sensor.hpp>

namespace concealer
{
NormalDistribution<sensor_msgs::msg::Imu>::NormalDistribution(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const std::string & topic)
: NormalDistributionBase(node, topic),
  // clang-format off
  orientation_r_error(        node, topic + ".sensor_msgs::msg::Imu.orientation.r.error"),
  orientation_p_error(        node, topic + ".sensor_msgs::msg::Imu.orientation.p.error"),
  orientation_y_error(        node, topic + ".sensor_msgs::msg::Imu.orientation.y.error"),
  angular_velocity_x_error(   node, topic + ".sensor_msgs::msg::Imu.angular_velocity.x.error"),
  angular_velocity_y_error(   node, topic + ".sensor_msgs::msg::Imu.angular_velocity.y.error"),
  angular_velocity_z_error(   node, topic + ".sensor_msgs::msg::Imu.angular_velocity.z.error"),
  linear_acceleration_x_error(node, topic + ".sensor_msgs::msg::Imu.linear_acceleration.x.error"),
  linear_acceleration_y_error(node, topic + ".sensor_msgs::msg::Imu.linear_acceleration.y.error"),
  linear_acceleration_z_error(node, topic + ".sensor_msgs::msg::Imu.linear_acceleration.z.error")
// clang-format on
{
}

auto NormalDistribution<sensor_msgs::msg::Imu>::deactivate() -> void
{
  orientation_r_error.active = false;
  orientation_p_error.active = false;
  orientation_y_error.active = false;
  angular_velocity_x_error.active = false;
  angular_velocity_y_error.active = false;
  angular_velocity_z_error.active = false;
  linear_acceleration_x_error.active = false;
  linear_acceleration_y_error.active = false;
  linear_acceleration_z_error.active = false;
}

auto NormalDistribution<sensor_msgs::msg::Imu>::operator()(sensor_msgs::msg::Imu imu)
  -> sensor_msgs::msg::Imu
{
  imu.orientation = math::geometry::convertEulerAngleToQuaternion([this, &imu] {
    auto rpy = math::geometry::convertQuaternionToEulerAngle(imu.orientation);

    rpy.x = orientation_r_error.apply(engine, rpy.x);
    rpy.y = orientation_p_error.apply(engine, rpy.y);
    rpy.z = orientation_y_error.apply(engine, rpy.z);
    return rpy;
  }());

  imu.angular_velocity.x = angular_velocity_x_error.apply(engine, imu.angular_velocity.x);
  imu.angular_velocity.y = angular_velocity_y_error.apply(engine, imu.angular_velocity.y);
  imu.angular_velocity.z = angular_velocity_z_error.apply(engine, imu.angular_velocity.z);

  imu.linear_acceleration.x = linear_acceleration_x_error.apply(engine, imu.linear_acceleration.x);
  imu.linear_acceleration.y = linear_acceleration_y_error.apply(engine, imu.linear_acceleration.y);
  imu.linear_acceleration.z = linear_acceleration_z_error.apply(engine, imu.linear_acceleration.z);

  return imu;
}
}  // namespace concealer

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

  if (not override_legacy_configuration_) {
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
