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

#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Geometry>
#include <concealer/publisher.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace concealer
{
RandomNumberEngine::RandomNumberEngine(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const std::string & topic)
: seed([&]() {
    if (const auto value = common::getParameter<int>(node, topic + ".seed");
        std::random_device::min() <= value and value <= std::random_device::max()) {
      return value;
    } else {
      throw common::scenario_simulator_exception::Error(
        "The value of parameter ", std::quoted(topic + ".seed"),
        " must be greater than or equal to ", std::random_device::min(),
        " and less than or equal to ", std::random_device::max());
    }
  }()),
  engine(seed ? seed : std::random_device()())
{
}

NormalDistribution<nav_msgs::msg::Odometry>::NormalDistribution(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const std::string & topic)
: RandomNumberEngine(node, topic),
  speed_threshold(
    common::getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.speed_threshold")),
  // clang-format off
  position_local_x_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.local_x.error"),
  position_local_y_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.local_y.error"),
  position_local_z_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.local_z.error"),
  orientation_r_error(   node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.r.error"),
  orientation_p_error(   node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.p.error"),
  orientation_y_error(   node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.y.error"),
  linear_x_error(        node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.x.error"),
  linear_y_error(        node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.y.error"),
  linear_z_error(        node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.z.error"),
  angular_x_error(       node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.x.error"),
  angular_y_error(       node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.y.error"),
  angular_z_error(       node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.z.error")
// clang-format on
{
}

auto NormalDistribution<nav_msgs::msg::Odometry>::operator()(nav_msgs::msg::Odometry odometry)
  -> nav_msgs::msg::Odometry
{
  geometry_msgs::msg::Pose & pose = odometry.pose.pose;
  geometry_msgs::msg::Twist & twist = odometry.twist.twist;

  if (const double speed = std::hypot(twist.linear.x, twist.linear.y, twist.linear.z);
      speed < speed_threshold) {
    return odometry;
  } else {
    const Eigen::Quaterniond orientation = Eigen::Quaterniond(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    Eigen::Vector3d local_position = Eigen::Vector3d(0.0, 0.0, 0.0);

    local_position.x() = position_local_x_error.apply(engine, local_position.x());
    local_position.y() = position_local_y_error.apply(engine, local_position.y());
    local_position.z() = position_local_z_error.apply(engine, local_position.z());

    const Eigen::Vector3d world_position = orientation.toRotationMatrix() * local_position;

    pose.position.x += world_position.x();
    pose.position.y += world_position.y();
    pose.position.z += world_position.z();

    Eigen::Vector3d euler = orientation.matrix().eulerAngles(0, 1, 2);

    euler.x() = orientation_r_error.apply(engine, euler.x());
    euler.y() = orientation_p_error.apply(engine, euler.y());
    euler.z() = orientation_y_error.apply(engine, euler.z());

    const Eigen::Quaterniond q = Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    twist.linear.x = linear_x_error.apply(engine, twist.linear.x);
    twist.linear.y = linear_y_error.apply(engine, twist.linear.y);
    twist.linear.z = linear_z_error.apply(engine, twist.linear.z);

    twist.angular.x = angular_x_error.apply(engine, twist.angular.x);
    twist.angular.y = angular_y_error.apply(engine, twist.angular.y);
    twist.angular.z = angular_z_error.apply(engine, twist.angular.z);

    return odometry;
  }
}

NormalDistribution<autoware_vehicle_msgs::msg::VelocityReport>::NormalDistribution(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const std::string & topic)
: RandomNumberEngine(node, topic),
  speed_threshold(common::getParameter<double>(
    node, topic + ".autoware_vehicle_msgs::msg::VelocityReport.speed_threshold")),
  // clang-format off
  longitudinal_velocity_error(node, topic + ".autoware_vehicle_msgs::msg::VelocityReport.longitudinal_velocity.error"),
  lateral_velocity_error(     node, topic + ".autoware_vehicle_msgs::msg::VelocityReport.lateral_velocity.error"),
  heading_rate_error(         node, topic + ".autoware_vehicle_msgs::msg::VelocityReport.heading_rate.error")
// clang-format on
{
}

auto NormalDistribution<autoware_vehicle_msgs::msg::VelocityReport>::operator()(
  autoware_vehicle_msgs::msg::VelocityReport velocity_report)
  -> autoware_vehicle_msgs::msg::VelocityReport
{
  if (const double speed =
        std::hypot(velocity_report.longitudinal_velocity, velocity_report.lateral_velocity);
      speed < speed_threshold) {
    return velocity_report;
  } else {
    velocity_report.longitudinal_velocity =
      longitudinal_velocity_error.apply(engine, velocity_report.longitudinal_velocity);
    velocity_report.lateral_velocity =
      lateral_velocity_error.apply(engine, velocity_report.lateral_velocity);
    velocity_report.heading_rate = heading_rate_error.apply(engine, velocity_report.heading_rate);
    return velocity_report;
  }
}

NormalDistribution<geometry_msgs::msg::PoseWithCovarianceStamped>::NormalDistribution(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const std::string & topic)
: RandomNumberEngine(node, topic),
  // clang-format off
  position_local_x_error(               node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.pose.position.local_x.error"),
  position_local_y_error(               node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.pose.position.local_y.error"),
  position_local_z_error(               node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.pose.position.local_z.error"),
  orientation_r_error(                  node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.pose.orientation.r.error"),
  orientation_p_error(                  node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.pose.orientation.p.error"),
  orientation_y_error(                  node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.pose.orientation.y.error"),
  covariance_diagonal_x_x_error(        node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.covariance.x_x.error"),
  covariance_diagonal_y_y_error(        node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.covariance.y_y.error"),
  covariance_diagonal_z_z_error(        node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.covariance.z_z.error"),
  covariance_diagonal_roll_roll_error(  node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.covariance.roll_roll.error"),
  covariance_diagonal_pitch_pitch_error(node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.covariance.pitch_pitch.error"),
  covariance_diagonal_yaw_yaw_error(    node, topic + ".geometry_msgs::msg::PoseWithCovarianceStamped.pose.covariance.yaw_yaw.error")
// clang-format on
{
}

auto NormalDistribution<geometry_msgs::msg::PoseWithCovarianceStamped>::operator()(
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped)
  -> geometry_msgs::msg::PoseWithCovarianceStamped
{
  geometry_msgs::msg::Pose & pose = pose_with_covariance_stamped.pose.pose;

  const Eigen::Quaterniond orientation = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

  Eigen::Vector3d local_position = Eigen::Vector3d(0.0, 0.0, 0.0);

  local_position.x() = position_local_x_error.apply(engine, local_position.x());
  local_position.y() = position_local_y_error.apply(engine, local_position.y());
  local_position.z() = position_local_z_error.apply(engine, local_position.z());

  const Eigen::Vector3d world_position = orientation.toRotationMatrix() * local_position;

  pose.position.x += world_position.x();
  pose.position.y += world_position.y();
  pose.position.z += world_position.z();

  tf2::Quaternion original_orientation;
  tf2::fromMsg(pose.orientation, original_orientation);

  double roll_error = orientation_r_error.apply(engine, 0.0);
  double pitch_error = orientation_p_error.apply(engine, 0.0);
  double yaw_error = orientation_y_error.apply(engine, 0.0);

  tf2::Quaternion error_quaternion;
  error_quaternion.setRPY(roll_error, pitch_error, yaw_error);

  tf2::Quaternion noised_orientation = original_orientation * error_quaternion;
  noised_orientation.normalize();

  pose.orientation = tf2::toMsg(noised_orientation);

  pose_with_covariance_stamped.pose.covariance.at(6 * 0 + 0) =
    covariance_diagonal_x_x_error.apply(engine, 0.0);
  pose_with_covariance_stamped.pose.covariance.at(6 * 1 + 1) =
    covariance_diagonal_y_y_error.apply(engine, 0.0);
  pose_with_covariance_stamped.pose.covariance.at(6 * 2 + 2) =
    covariance_diagonal_z_z_error.apply(engine, 0.0);
  pose_with_covariance_stamped.pose.covariance.at(6 * 3 + 3) =
    covariance_diagonal_roll_roll_error.apply(engine, 0.0);
  pose_with_covariance_stamped.pose.covariance.at(6 * 4 + 4) =
    covariance_diagonal_pitch_pitch_error.apply(engine, 0.0);
  pose_with_covariance_stamped.pose.covariance.at(6 * 5 + 5) =
    covariance_diagonal_yaw_yaw_error.apply(engine, 0.0);

  return pose_with_covariance_stamped;
}

NormalDistribution<sensor_msgs::msg::Imu>::NormalDistribution(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const std::string & topic)
: RandomNumberEngine(node, topic),
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
