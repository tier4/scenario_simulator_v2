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

#include <Eigen/Geometry>
#include <concealer/publisher.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace concealer
{
NormalDistribution<nav_msgs::msg::Odometry>::NormalDistribution(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const std::string & topic)
: seed([&]() {
    if (const auto value = getParameter<int>(node, topic + ".seed");
        std::random_device::min() <= value and value <= std::random_device::max()) {
      return value;
    } else {
      throw common::scenario_simulator_exception::Error(
        "The value of parameter ", std::quoted(topic + ".seed"),
        " must be greater than or equal to ", std::random_device::min(),
        " and less than or equal to ", std::random_device::max());
    }
  }()),
  engine(seed ? seed : device()),
  position_local_x_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.local_x.error"),
  position_local_y_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.local_y.error"),
  position_local_z_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.local_z.error"),
  orientation_r_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.r.error"),
  orientation_p_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.p.error"),
  orientation_y_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.y.error"),
  linear_x_error(node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.x.error"),
  linear_y_error(node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.y.error"),
  linear_z_error(node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.z.error"),
  angular_x_error(node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.x.error"),
  angular_y_error(node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.y.error"),
  angular_z_error(node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.z.error")
{
}

auto NormalDistribution<nav_msgs::msg::Odometry>::operator()(nav_msgs::msg::Odometry odometry)
  -> nav_msgs::msg::Odometry
{
  const Eigen::Quaterniond orientation = Eigen::Quaterniond(
    odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x,
    odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z);

  Eigen::Vector3d local_position = Eigen::Vector3d(0.0, 0.0, 0.0);

  local_position.x() = position_local_x_error.apply(engine, local_position.x());
  local_position.y() = position_local_y_error.apply(engine, local_position.y());
  local_position.z() = position_local_z_error.apply(engine, local_position.z());

  const Eigen::Vector3d world_position = orientation.toRotationMatrix() * local_position;

  odometry.pose.pose.position.x += world_position.x();
  odometry.pose.pose.position.y += world_position.y();
  odometry.pose.pose.position.z += world_position.z();

  Eigen::Vector3d euler = orientation.matrix().eulerAngles(0, 1, 2);

  euler.x() = orientation_r_error.apply(engine, euler.x());
  euler.y() = orientation_p_error.apply(engine, euler.y());
  euler.z() = orientation_y_error.apply(engine, euler.z());

  const Eigen::Quaterniond q = Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());

  odometry.pose.pose.orientation.x = q.x();
  odometry.pose.pose.orientation.y = q.y();
  odometry.pose.pose.orientation.z = q.z();
  odometry.pose.pose.orientation.w = q.w();

  odometry.twist.twist.linear.x = linear_x_error.apply(engine, odometry.twist.twist.linear.x);
  odometry.twist.twist.linear.y = linear_y_error.apply(engine, odometry.twist.twist.linear.y);
  odometry.twist.twist.linear.z = linear_z_error.apply(engine, odometry.twist.twist.linear.z);

  odometry.twist.twist.angular.x = angular_x_error.apply(engine, odometry.twist.twist.angular.x);
  odometry.twist.twist.angular.y = angular_y_error.apply(engine, odometry.twist.twist.angular.y);
  odometry.twist.twist.angular.z = angular_z_error.apply(engine, odometry.twist.twist.angular.z);

  return odometry;
}
}  // namespace concealer
