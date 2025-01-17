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
: seed(getParameter<int>(node, topic + ".seed")),
  engine(seed ? seed : device()),
  position_x_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.x.error"),
  position_y_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.y.error"),
  position_z_error(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.z.error"),
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
  position_x_error.apply(engine, odometry.pose.pose.position.x);
  position_y_error.apply(engine, odometry.pose.pose.position.y);
  position_z_error.apply(engine, odometry.pose.pose.position.z);

  Eigen::Vector3d euler = Eigen::Quaterniond(
                            odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x,
                            odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z)
                            .matrix()
                            .eulerAngles(0, 1, 2);

  orientation_r_error.apply(engine, euler.x());
  orientation_p_error.apply(engine, euler.y());
  orientation_y_error.apply(engine, euler.z());

  const Eigen::Quaterniond orientation = Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()) *
                                         Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());

  odometry.pose.pose.orientation.x = orientation.x();
  odometry.pose.pose.orientation.y = orientation.y();
  odometry.pose.pose.orientation.z = orientation.z();
  odometry.pose.pose.orientation.w = orientation.w();

  linear_x_error.apply(engine, odometry.twist.twist.linear.x);
  linear_y_error.apply(engine, odometry.twist.twist.linear.y);
  linear_z_error.apply(engine, odometry.twist.twist.linear.z);

  angular_x_error.apply(engine, odometry.twist.twist.angular.x);
  angular_y_error.apply(engine, odometry.twist.twist.angular.y);
  angular_z_error.apply(engine, odometry.twist.twist.angular.z);

  return odometry;
}
}  // namespace concealer
