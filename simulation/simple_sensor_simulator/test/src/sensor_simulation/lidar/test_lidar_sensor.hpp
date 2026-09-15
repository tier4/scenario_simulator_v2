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

#include <agnocast_wrapper/agnocast_wrapper.hpp>
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
  /*
     There are two fixtures below, so gtest calls these twice. The context is never shut down:
     shutting it down while the static node of `common::getParameterNode()` still exists terminates
     the process during static destruction.
  */
  static void SetUpTestSuite()
  {
    if (not rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {}

  explicit LidarSensorTest(
    const std::string & topic_name = "lidar_output",
    const std::string & architecture_type = "awf/universe/20240605")
  : config_(utils::constructLidarConfiguration("ego", architecture_type, 0.0, 0.5))
  {
    // Note: Executor must be created after rclcpp::init. If created before, it causes context is null error.
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    node_ = std::make_shared<rclcpp::Node>("lidar_sensor_test_node_" + topic_name);
    executor_->add_node(node_);
    makeRosInterface(topic_name);
    initializeEntityStatuses();

    lidar_ = std::make_unique<LidarSensor<sensor_msgs::msg::PointCloud2>>(0.0, config_, publisher_);
  }

  ~LidarSensorTest() = default;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  agnocast_wrapper::PublisherPtr<sensor_msgs::msg::PointCloud2> publisher_;
  agnocast_wrapper::SubscriptionPtr<sensor_msgs::msg::PointCloud2> subscription_;

  std::vector<EntityStatus> status_;

  std::unique_ptr<LidarSensorBase> lidar_;
  simulation_api_schema::LidarConfiguration config_;
  agnocast_wrapper::MessagePtr<sensor_msgs::msg::PointCloud2> received_msg_;

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

    /*
       A misc object is the only entity whose classification is not object compatible, so it is the
       only one whose points reach the segmented topic.
    */
    const auto other3_status = utils::makeEntity(
      "other3", EntityType::MISC_OBJECT, utils::makePose(0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      dimensions);

    status_ = {ego_status, other1_status, other2_status, other3_status};
  }

  auto makeRosInterface(const std::string & topic_name) -> void
  {
    publisher_ =
      agnocast_wrapper::create_publisher<sensor_msgs::msg::PointCloud2>(node_, topic_name, 10);
    subscription_ = agnocast_wrapper::create_subscription<sensor_msgs::msg::PointCloud2>(
      node_, topic_name, 10,
      [this](const agnocast_wrapper::MessagePtr<sensor_msgs::msg::PointCloud2> msg) {
        received_msg_ = msg;
      });
  }
};

/// @brief A LiDAR sensor publishing `PointXYZCPE`, whose classes come from the entities.
class SegmentedLidarSensorTest : public LidarSensorTest
{
protected:
  SegmentedLidarSensorTest() : LidarSensorTest("lidar_output_segmented", "awf/universe/20260801") {}
};
#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_LIDAR_SENSOR_HPP_
