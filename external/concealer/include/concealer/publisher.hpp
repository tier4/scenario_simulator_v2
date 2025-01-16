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

#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

namespace concealer
{
template <typename>
struct Identity
{
  template <typename... Ts>
  explicit constexpr Identity(Ts &&...)
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

  // clang-format off
  std::normal_distribution<double> position_x,
                                   position_y,
                                   position_z,
                                   orientation_r,
                                   orientation_p,
                                   orientation_y,
                                   linear_x,
                                   linear_y,
                                   linear_z,
                                   angular_x,
                                   angular_y,
                                   angular_z;
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
