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
  : entity_name_(configuration.entity()),
    add_noise_(configuration.add_noise()),
    noise_standard_deviation_(configuration.noise_standard_deviation()),
    random_generator_(configuration.use_seed() ? configuration.seed() : std::random_device{}()),
    noise_distribution_(0.0, noise_standard_deviation_)
  {
  }

  virtual ~ImuSensorBase() = default;

  virtual auto update(
    const rclcpp::Time & current_ros_time,
    const std::vector<traffic_simulator_msgs::EntityStatus> & statuses) const -> bool = 0;

protected:
  const std::string entity_name_;
  const bool add_noise_;
  const double noise_standard_deviation_;
  mutable std::mt19937 random_generator_;
  mutable std::normal_distribution<> noise_distribution_;
};

template <typename MessageType>
class ImuSensor : public ImuSensorBase
{
public:
  explicit ImuSensor(
    const simulation_api_schema::ImuSensorConfiguration & configuration,
    const typename rclcpp::Publisher<MessageType>::SharedPtr & publisher)
  : ImuSensorBase(configuration), publisher_(publisher)
  {
  }

  auto update(
    const rclcpp::Time & current_ros_time,
    const std::vector<traffic_simulator_msgs::EntityStatus> & statuses) const -> bool
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
    const traffic_simulator_msgs::msg::EntityStatus & status) const -> const sensor_msgs::msg::Imu;

  const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};
}  // namespace simple_sensor_simulator
#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__IMU_SENSOR_HPP_
