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
template <typename T>
auto getParameter(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const std::string & name, T value = {})
{
  if (not node->has_parameter(name)) {
    node->declare_parameter(name, rclcpp::ParameterValue(value));
  }
  return node->get_parameter(name).get_value<T>();
}

NormalDistribution<nav_msgs::msg::Odometry>::NormalDistribution(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const std::string & topic)
: seed(getParameter<int>(node, topic + ".seed")),
  engine(seed ? seed : device()),
  position_x(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.x.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.x.standard_deviation")),
  position_y(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.y.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.y.standard_deviation")),
  position_z(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.z.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.pose.pose.position.z.standard_deviation")),
  orientation_r(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.r.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.r.standard_deviation")),
  orientation_p(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.p.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.p.standard_deviation")),
  orientation_y(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.y.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.pose.pose.orientation.y.standard_deviation")),
  linear_x(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.x.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.x.standard_deviation")),
  linear_y(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.y.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.y.standard_deviation")),
  linear_z(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.z.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.twist.twist.linear.z.standard_deviation")),
  angular_x(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.x.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.x.standard_deviation")),
  angular_y(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.y.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.y.standard_deviation")),
  angular_z(
    getParameter<double>(node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.z.mean"),
    getParameter<double>(
      node, topic + ".nav_msgs::msg::Odometry.twist.twist.angular.z.standard_deviation"))
{
}

auto NormalDistribution<nav_msgs::msg::Odometry>::operator()(nav_msgs::msg::Odometry odometry)
  -> nav_msgs::msg::Odometry
{
  odometry.pose.pose.position.x += position_x(engine);
  odometry.pose.pose.position.y += position_y(engine);
  odometry.pose.pose.position.z += position_z(engine);

  Eigen::Vector3d euler = Eigen::Quaterniond(
                            odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x,
                            odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z)
                            .matrix()
                            .eulerAngles(0, 1, 2);

  euler.x() += orientation_r(engine);
  euler.y() += orientation_p(engine);
  euler.z() += orientation_y(engine);

  const Eigen::Quaterniond orientation = Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()) *
                                         Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());

  odometry.pose.pose.orientation.x = orientation.x();
  odometry.pose.pose.orientation.y = orientation.y();
  odometry.pose.pose.orientation.z = orientation.z();
  odometry.pose.pose.orientation.w = orientation.w();

  odometry.twist.twist.linear.x += linear_x(engine);
  odometry.twist.twist.linear.y += linear_y(engine);
  odometry.twist.twist.linear.z += linear_z(engine);

  odometry.twist.twist.angular.x += angular_x(engine);
  odometry.twist.twist.angular.y += angular_y(engine);
  odometry.twist.twist.angular.z += angular_z(engine);

  return odometry;
}
}  // namespace concealer
