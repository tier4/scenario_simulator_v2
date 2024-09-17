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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <set>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <string>
#include <vector>

#include "../../utils/helper_functions.hpp"

using namespace simple_sensor_simulator;

class LidarSensorTest : public ::testing::Test
{
protected:
  LidarSensorTest() : config_(utils::constructLidarConfiguration("ego", "awf/universe", 0.0, 0.5))
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("lidar_sensor_test_node");
    makeRosInterface();
    initializeEntityStatuses();

    lidar_ = std::make_unique<LidarSensor<sensor_msgs::msg::PointCloud2>>(0.0, config_, publisher_);
  }

  ~LidarSensorTest() { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  std::vector<EntityStatus> status_;

  std::unique_ptr<LidarSensorBase> lidar_;
  simulation_api_schema::LidarConfiguration config_;
  sensor_msgs::msg::PointCloud2::SharedPtr received_msg_;

  double current_simulation_time_{1.0};
  rclcpp::Time current_ros_time_{1};

private:
  auto initializeEntityStatuses() -> void
  {
    const auto dimensions = utils::makeDimensions(4.5, 2.0, 1.5);

    const auto ego_status = utils::makeEntity(
      "ego", EntityType::EGO, utils::makePose(5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0), dimensions);
    const auto other1_status = utils::makeEntity(
      "other1", EntityType::VEHICLE, utils::makePose(-3.0, -3.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      dimensions);
    const auto other2_status = utils::makeEntity(
      "other2", EntityType::VEHICLE, utils::makePose(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      dimensions);

    status_ = {ego_status, other1_status, other2_status};
  }

  auto makeRosInterface() -> void
  {
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_output", 10);
    subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "lidar_output", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { received_msg_ = msg; });
  }
};
#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_LIDAR_SENSOR_HPP_
