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

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <get_parameter/get_parameter.hpp>
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

template <typename ValueType>
struct NormalDistributionError
{
  static_assert(
    std::disjunction_v<
      std::is_same<std::decay_t<ValueType>, float>, std::is_same<std::decay_t<ValueType>, double>,
      std::is_same<std::decay_t<ValueType>, long double> >,
    "Unsupported error type");

  std::normal_distribution<ValueType> additive, multiplicative;

  explicit NormalDistributionError(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
    const std::string & prefix)
  // clang-format off
  : additive(
      static_cast<ValueType>(common::getParameter<double>(node, prefix + ".additive.mean")),
      static_cast<ValueType>(common::getParameter<double>(node, prefix + ".additive.standard_deviation"))),
    multiplicative(
      static_cast<ValueType>(common::getParameter<double>(node, prefix + ".multiplicative.mean")),
      static_cast<ValueType>(common::getParameter<double>(node, prefix + ".multiplicative.standard_deviation")))
  // clang-format on
  {
  }

  auto apply(std::mt19937_64 & engine, const ValueType value) -> decltype(auto)
  {
    return value * (multiplicative(engine) + static_cast<ValueType>(1)) + additive(engine);
  }
};

template <typename>
struct NormalDistribution;

/**
 * @brief Provides common components for obtaining the seed and initializing the pseudo random number generator engine
 * obtains the seed from the parameter <topic>.seed
 * initializes `engine` appropriately
 */
struct NormalDistributionBase
{
  const std::random_device::result_type seed;

  std::mt19937_64 engine;

  NormalDistributionBase(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
    const std::string & topic);
};

template <>
struct NormalDistribution<nav_msgs::msg::Odometry> : public NormalDistributionBase
{
  const double speed_threshold;

  // clang-format off
  NormalDistributionError<double> position_local_x_error,
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

template <>
struct NormalDistribution<autoware_vehicle_msgs::msg::VelocityReport>
: public NormalDistributionBase
{
  const double speed_threshold;

  // clang-format off
  NormalDistributionError<float> longitudinal_velocity_error,
                                 lateral_velocity_error,
                                 heading_rate_error;
  // clang-format on

  explicit NormalDistribution(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &, const std::string &);

  auto operator()(autoware_vehicle_msgs::msg::VelocityReport velocity_report)
    -> autoware_vehicle_msgs::msg::VelocityReport;
};

template <>
struct NormalDistribution<geometry_msgs::msg::PoseWithCovarianceStamped>
: public NormalDistributionBase
{
  // clang-format off
  NormalDistributionError<double> position_local_x_error,
                                  position_local_y_error,
                                  position_local_z_error,
                                  orientation_r_error,
                                  orientation_p_error,
                                  orientation_y_error;
  // clang-format on

  explicit NormalDistribution(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &, const std::string &);

  auto operator()(geometry_msgs::msg::PoseWithCovarianceStamped pose)
    -> geometry_msgs::msg::PoseWithCovarianceStamped;
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
