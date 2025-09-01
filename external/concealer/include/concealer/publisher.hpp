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

#include <get_parameter/get_parameter.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

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

  double speed_threshold;

  struct Error
  {
    std::normal_distribution<double> additive, multiplicative;

    explicit Error(
      const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
      const std::string & prefix)
    : additive(
        common::getParameter<double>(node, prefix + ".additive.mean"),
        common::getParameter<double>(node, prefix + ".additive.standard_deviation")),
      multiplicative(
        common::getParameter<double>(node, prefix + ".multiplicative.mean"),
        common::getParameter<double>(node, prefix + ".multiplicative.standard_deviation"))
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

  std::string topic_name;
  
  mutable std::size_t message_count = 0;
  mutable std::chrono::steady_clock::time_point last_message_time = std::chrono::steady_clock::now();
  mutable std::chrono::steady_clock::time_point first_message_time;
  mutable bool first_message_sent = false;

public:
  template <typename Node>
  explicit Publisher(const std::string & topic, Node & node)
  : publisher(node.template create_publisher<Message>(topic, rclcpp::QoS(1).reliable())),
    randomize(node.get_node_parameters_interface(), topic),
    topic_name(topic)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("DEBUG/concealer::Publisher"),
      "Created publisher for topic %s", topic.c_str());
  }

  template <typename... Ts>
  auto operator()(Ts &&... xs) -> decltype(auto)
  {
    ++message_count;
    
    if (message_count <= 3 || (message_count % 50 == 0 && message_count<=150)) {
      auto now = std::chrono::steady_clock::now();
      
      if (!first_message_sent) {
        first_message_time = now;
        first_message_sent = true;
      }
      
      double frequency = 0.0;
      if (message_count > 1) {
        auto time_diff = std::chrono::duration<double>(now - first_message_time).count();
        frequency = (message_count - 1) / time_diff;
      }
      
      RCLCPP_WARN(
        rclcpp::get_logger("DEBUG/concealer::Publisher"),
        "Publishing message [no. %zu] on topic %s (freq: %.2f Hz)", message_count, topic_name.c_str(), frequency);
        
      last_message_time = now;
    }
    
    return publisher->publish(randomize(std::forward<decltype(xs)>(xs)...));
  }
};
}  // namespace concealer

#endif  // CONCEALER__PUBLISHER_HPP_
