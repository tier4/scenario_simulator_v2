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

#ifndef SIMPLE_SENSOR_SIMULATOR__TEST__TEST_LIDAR_SENSOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__TEST__TEST_LIDAR_SENSOR_HPP_

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <set>
#include <simple_sensor_simulator/sensor_simulation/imu/imu_sensor.hpp>
#include <string>
#include <vector>

#include "../../utils/helper_functions.hpp"

using namespace simple_sensor_simulator;

class ImuSensorTest : public ::testing::Test
{
protected:
  ImuSensorTest()
  : config_([] {
      simulation_api_schema::ImuSensorConfiguration config;

      config.set_entity("ego");
      config.set_frame_id("imu_frame");
      config.set_add_gravity(true);
      config.set_use_seed(true);
      config.set_seed(1);
      config.set_noise_standard_deviation_orientation(2);
      config.set_noise_standard_deviation_twist(3);
      config.set_noise_standard_deviation_acceleration(4);

      return config;
    }())
  {
    rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("imu_sensor_test_node");

    subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "imu_output", 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { received_msg_ = msg; });
  }

  auto initializeSensor() -> void
  {
    initializeEntityStatuses();

    imu_ = std::make_unique<ImuSensor<sensor_msgs::msg::Imu>>(config_, "imu_output", *node_);
  }

  ~ImuSensorTest() { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;

  simulation_api_schema::ImuSensorConfiguration config_;
  std::unique_ptr<ImuSensorBase> imu_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;

  std::vector<EntityStatus> status_;

  sensor_msgs::msg::Imu::SharedPtr received_msg_;

  double current_simulation_time_{1.0};
  rclcpp::Time current_ros_time_{1};

private:
  auto initializeEntityStatuses() -> void
  {
    const auto dimensions = utils::makeDimensions(4.5, 2.0, 1.5);

    auto ego_status = utils::makeEntity(
      "ego", EntityType::EGO, utils::makePose(5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0), dimensions);

    ego_status.mutable_action_status()->mutable_twist()->mutable_angular()->set_z(0.5);
    ego_status.mutable_action_status()->mutable_accel()->mutable_linear()->set_x(1.2);

    const auto other1_status = utils::makeEntity(
      "other1", EntityType::VEHICLE, utils::makePose(-3.0, -3.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      dimensions);
    const auto other2_status = utils::makeEntity(
      "other2", EntityType::VEHICLE, utils::makePose(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      dimensions);

    status_ = {ego_status, other1_status, other2_status};
  }
};
#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_LIDAR_SENSOR_HPP_
