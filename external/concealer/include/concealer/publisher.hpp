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

#ifndef CONCEALER__PUBLISHER_HPP_
#define CONCEALER__PUBLISHER_HPP_

#include <concealer/get_parameter.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

namespace concealer
{
template <typename>
struct Identity
{
  explicit constexpr Identity(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &, const std::string &)
  {
  }

  template <typename T>
  constexpr auto operator()(T && x) const -> decltype(auto)
  {
    return std::forward<decltype(x)>(x);
  }
};

template <typename>
struct NormalDistribution;

template <>
struct NormalDistribution<nav_msgs::msg::Odometry>
{
  std::random_device::result_type seed;

  std::random_device device;

  std::mt19937_64 engine;

  struct Error
  {
    std::normal_distribution<double> additive, multiplicative;

    explicit Error(
      const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
      const std::string & prefix)
    : additive(
        getParameter<double>(node, prefix + ".additive.mean"),
        getParameter<double>(node, prefix + ".additive.standard_deviation")),
      multiplicative(
        getParameter<double>(node, prefix + ".multiplicative.mean"),
        getParameter<double>(node, prefix + ".multiplicative.standard_deviation"))
    {
    }

    auto apply(std::mt19937_64 & engine, double value) -> decltype(auto)
    {
      return value * (multiplicative(engine) + 1.0) + additive(engine);
    }
  };

  // clang-format off
  Error position_local_x_error,
        position_local_y_error,
        position_local_z_error,
        orientation_r_error,
        orientation_p_error,
        orientation_y_error,
        linear_x_error,
        linear_y_error,
        linear_z_error,
        angular_x_error,
        angular_y_error,
        angular_z_error;
  // clang-format on

  explicit NormalDistribution(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &, const std::string &);

  auto operator()(nav_msgs::msg::Odometry odometry) -> nav_msgs::msg::Odometry;
};

template <typename Message, template <typename> typename Randomizer = Identity>
class Publisher
{
  typename rclcpp::Publisher<Message>::SharedPtr publisher;

  Randomizer<Message> randomize;

public:
  template <typename Node>
  explicit Publisher(const std::string & topic, Node & node)
  : publisher(node.template create_publisher<Message>(topic, rclcpp::QoS(1).reliable())),
    randomize(node.get_node_parameters_interface(), topic)
  {
  }

  template <typename... Ts>
  auto operator()(Ts &&... xs) -> decltype(auto)
  {
    return publisher->publish(randomize(std::forward<decltype(xs)>(xs)...));
  }
};
}  // namespace concealer

#endif  // CONCEALER__PUBLISHER_HPP_
