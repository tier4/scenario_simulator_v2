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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__IMU_SENSOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__IMU_SENSOR_HPP_

#include <simulation_api_schema.pb.h>

#include <array>
#include <boost/math/constants/constants.hpp>
#include <concealer/publisher.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <simulation_interface/conversions.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>

namespace concealer
{
template <>
struct NormalDistribution<sensor_msgs::msg::Imu> : public RandomNumberEngine
{
  // clang-format off
  NormalDistributionError<double> orientation_r_error,
                                  orientation_p_error,
                                  orientation_y_error,
                                  angular_velocity_x_error,
                                  angular_velocity_y_error,
                                  angular_velocity_z_error,
                                  linear_acceleration_x_error,
                                  linear_acceleration_y_error,
                                  linear_acceleration_z_error;
  // clang-format on

  explicit NormalDistribution(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr &, const std::string &);

  auto deactivate() -> void;

  auto operator()(sensor_msgs::msg::Imu imu) -> sensor_msgs::msg::Imu;
};
}  // namespace concealer

namespace simple_sensor_simulator
{
class ImuSensorBase
{
public:
  explicit ImuSensorBase(const simulation_api_schema::ImuSensorConfiguration & configuration)
  : add_gravity_(configuration.add_gravity()),
    noise_standard_deviation_orientation_(configuration.noise_standard_deviation_orientation()),
    noise_standard_deviation_twist_(configuration.noise_standard_deviation_twist()),
    noise_standard_deviation_acceleration_(configuration.noise_standard_deviation_acceleration()),
    random_generator_(configuration.use_seed() ? configuration.seed() : std::random_device{}()),
    noise_distribution_orientation_(0.0, noise_standard_deviation_orientation_),
    noise_distribution_twist_(0.0, noise_standard_deviation_twist_),
    noise_distribution_acceleration_(0.0, noise_standard_deviation_acceleration_),
    orientation_covariance_(calculateCovariance(noise_standard_deviation_orientation_)),
    angular_velocity_covariance_(calculateCovariance(noise_standard_deviation_twist_)),
    linear_acceleration_covariance_(calculateCovariance(noise_standard_deviation_acceleration_))
  {
  }

  virtual ~ImuSensorBase() = default;

  virtual auto update(
    const rclcpp::Time & current_ros_time,
    const std::vector<traffic_simulator_msgs::EntityStatus> & statuses) const -> bool = 0;

protected:
  const bool add_gravity_;
  const double noise_standard_deviation_orientation_;
  const double noise_standard_deviation_twist_;
  const double noise_standard_deviation_acceleration_;
  mutable std::mt19937 random_generator_;
  mutable std::normal_distribution<> noise_distribution_orientation_;
  mutable std::normal_distribution<> noise_distribution_twist_;
  mutable std::normal_distribution<> noise_distribution_acceleration_;
  std::array<double, 9> orientation_covariance_;
  std::array<double, 9> angular_velocity_covariance_;
  std::array<double, 9> linear_acceleration_covariance_;

  auto calculateCovariance(const double stddev) const -> std::array<double, 9>
  {
    return {std::pow(stddev, 2), 0, 0, 0, std::pow(stddev, 2), 0, 0, 0, std::pow(stddev, 2)};
  }

  auto calculateCovariance(const double variance0, const double variance1, const double variance2)
    const -> std::array<double, 9>
  {
    return {variance0, 0, 0, 0, variance1, 0, 0, 0, variance2};
  };
};

template <typename MessageType>
class ImuSensor : public ImuSensorBase
{
public:
  template <typename NodeType>
  explicit ImuSensor(
    const simulation_api_schema::ImuSensorConfiguration & configuration, const std::string & topic,
    NodeType & node)
  : ImuSensorBase(configuration),
    override_legacy_configuration_(common::getParameter<bool>(
      node.get_node_parameters_interface(), topic + ".override_legacy_configuration", false)),
    entity_name_(configuration.entity()),
    frame_id_(configuration.frame_id()),
    publish(topic, node)
  {
    /**
     * @note By default publisher randomization will be active, so we need to deactivate it
     * If legacy is not overriden we don't want to recalculate covariance matrices, so return early
     */
    if (not override_legacy_configuration_) {
      publish.getMutableRandomizer().deactivate();
      return;
    }

    /**
     * @note Calculate covariance matrices based on some nominal values
     * These values have no technical reason, they are an educated guess of what is reasonable
     */
    constexpr double nominal_angle{boost::math::constants::quarter_pi<double>()};
    // clang-format off
    orientation_covariance_ = calculateCovariance(
      calculateVariance(nominal_angle,
        publish.getRandomizer().orientation_r_error.multiplicative.stddev(),
        publish.getRandomizer().orientation_r_error.additive.stddev()),
      calculateVariance(nominal_angle,
        publish.getRandomizer().orientation_p_error.multiplicative.stddev(),
        publish.getRandomizer().orientation_p_error.additive.stddev()),
      calculateVariance(nominal_angle,
        publish.getRandomizer().orientation_y_error.multiplicative.stddev(),
        publish.getRandomizer().orientation_y_error.additive.stddev()));
    // clang-format on

    constexpr double nominal_velocity{5.0};
    // clang-format off
    angular_velocity_covariance_ = calculateCovariance(
      calculateVariance(nominal_velocity,
        publish.getRandomizer().angular_velocity_x_error.multiplicative.stddev(),
        publish.getRandomizer().angular_velocity_x_error.additive.stddev()),
      calculateVariance(nominal_velocity,
        publish.getRandomizer().angular_velocity_y_error.multiplicative.stddev(),
        publish.getRandomizer().angular_velocity_y_error.additive.stddev()),
      calculateVariance(nominal_velocity,
        publish.getRandomizer().angular_velocity_z_error.multiplicative.stddev(),
        publish.getRandomizer().angular_velocity_z_error.additive.stddev()));
    // clang-format on

    constexpr double nominal_acceleration{0.5};
    // clang-format off
    linear_acceleration_covariance_ = calculateCovariance(
      calculateVariance(nominal_acceleration,
        publish.getRandomizer().linear_acceleration_x_error.multiplicative.stddev(),
        publish.getRandomizer().linear_acceleration_x_error.additive.stddev()),
      calculateVariance(nominal_acceleration,
        publish.getRandomizer().linear_acceleration_y_error.multiplicative.stddev(),
        publish.getRandomizer().linear_acceleration_y_error.additive.stddev()),
      calculateVariance(nominal_acceleration,
        publish.getRandomizer().linear_acceleration_z_error.multiplicative.stddev(),
        publish.getRandomizer().linear_acceleration_z_error.additive.stddev()));
    // clang-format on
  }

  auto update(
    const rclcpp::Time & current_ros_time,
    const std::vector<traffic_simulator_msgs::EntityStatus> & statuses) const -> bool override
  {
    for (const auto & status : statuses) {
      if (status.name() == entity_name_) {
        traffic_simulator_msgs::msg::EntityStatus status_msg;
        simulation_interface::toMsg(status, status_msg);
        publish(generateMessage(current_ros_time, status_msg));
        return true;
      }
    }
    return false;
  }

private:
  auto generateMessage(
    const rclcpp::Time & current_ros_time,
    const traffic_simulator_msgs::msg::EntityStatus & status) const -> const MessageType;

  static auto calculateVariance(
    const double nominal_value, const double multiplicative_stddev, const double additive_stddev)
    -> double
  {
    return std::pow(nominal_value, 2) * std::pow(multiplicative_stddev, 2) +
           std::pow(additive_stddev, 2);
  }

  const bool override_legacy_configuration_;
  const std::string entity_name_;
  const std::string frame_id_;
  const concealer::Publisher<MessageType, concealer::NormalDistribution> publish;
};
}  // namespace simple_sensor_simulator
#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__IMU_SENSOR_HPP_
