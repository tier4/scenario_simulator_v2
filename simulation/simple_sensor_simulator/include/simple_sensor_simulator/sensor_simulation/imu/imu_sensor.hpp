// Copyright 2024 TIER IV, Inc. All rights reserved.
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
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <simulation_interface/conversions.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>

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
  const std::array<double, 9> orientation_covariance_;
  const std::array<double, 9> angular_velocity_covariance_;
  const std::array<double, 9> linear_acceleration_covariance_;

  auto calculateCovariance(const double stddev) const -> std::array<double, 9>
  {
    return {std::pow(stddev, 2), 0, 0, 0, std::pow(stddev, 2), 0, 0, 0, std::pow(stddev, 2)};
  };
};

template <typename MessageType>
class ImuSensor : public ImuSensorBase
{
public:
  explicit ImuSensor(
    const simulation_api_schema::ImuSensorConfiguration & configuration,
    const typename rclcpp::Publisher<MessageType>::SharedPtr & publisher)
  : ImuSensorBase(configuration),
    entity_name_(configuration.entity()),
    frame_id_(configuration.frame_id()),
    publisher_(publisher)
  {
  }

  auto update(
    const rclcpp::Time & current_ros_time,
    const std::vector<traffic_simulator_msgs::EntityStatus> & statuses) const -> bool override
  {
    for (const auto & status : statuses) {
      if (status.name() == entity_name_) {
        traffic_simulator_msgs::msg::EntityStatus status_msg;
        simulation_interface::toMsg(status, status_msg);
        publisher_->publish(generateMessage(current_ros_time, status_msg));
        return true;
      }
    }
    return false;
  }

private:
  auto generateMessage(
    const rclcpp::Time & current_ros_time,
    const traffic_simulator_msgs::msg::EntityStatus & status) const -> const MessageType;

  const std::string entity_name_;
  const std::string frame_id_;
  const typename rclcpp::Publisher<MessageType>::SharedPtr publisher_;
};
}  // namespace simple_sensor_simulator
#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__IMU_SENSOR_HPP_
