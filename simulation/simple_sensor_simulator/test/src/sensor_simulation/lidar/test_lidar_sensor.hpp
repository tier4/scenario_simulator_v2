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
#include <simulation_api_schema.pb.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <set>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <string>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <vector>

using namespace simple_sensor_simulator;

class LidarSensorTest : public ::testing::Test
{
protected:
  LidarSensorTest()
  {
    configureLidarSensor();
    initializeEntityStatuses();
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("lidar_sensor_test_node");
    createRosInterface();

    lidar_ = std::make_unique<LidarSensor<sensor_msgs::msg::PointCloud2>>(0.0, config_, publisher_);
  }
  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  traffic_simulator_msgs::EntityStatus ego_status_;
  traffic_simulator_msgs::EntityStatus other1_status_;
  traffic_simulator_msgs::EntityStatus other2_status_;
  std::vector<traffic_simulator_msgs::EntityStatus> status_;

  std::unique_ptr<LidarSensorBase> lidar_;
  simulation_api_schema::LidarConfiguration config_;
  sensor_msgs::msg::PointCloud2::SharedPtr received_msg_;

  double current_simulation_time_{1.0};
  rclcpp::Time current_ros_time_{1};

private:
  auto configureLidarSensor() -> void
  {
    config_.set_entity("ego");
    config_.set_architecture_type("awf/universe");
    config_.set_scan_duration(0.1);
    config_.set_lidar_sensor_delay(0.0);

    for (double angle = -15.0; angle <= 15.0; angle += 2.0) {
      config_.add_vertical_angles(angle * M_PI / 180.0);
    }
    config_.set_horizontal_resolution(0.5 * M_PI / 180.0);
  }

  auto initializeEntityStatuses() -> void
  {
    auto dimensions = createDimensions(4.5, 2.0, 1.5);

    initializeEntityStatus(
      ego_status_, "ego", traffic_simulator_msgs::EntityType::EGO,
      createPose(5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0), dimensions);
    initializeEntityStatus(
      other1_status_, "other1", traffic_simulator_msgs::EntityType::VEHICLE,
      createPose(-3.0, -3.0, 0.0, 0.0, 0.0, 0.0, 1.0), dimensions);
    initializeEntityStatus(
      other2_status_, "other2", traffic_simulator_msgs::EntityType::VEHICLE,
      createPose(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), dimensions);

    status_ = {ego_status_, other1_status_, other2_status_};
  }

  auto createPose(double px, double py, double pz, double ox, double oy, double oz, double ow)
    -> geometry_msgs::msg::Pose
  {
    return geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(px).y(py).z(pz))
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(ox).y(oy).z(oz).w(ow));
  }

  auto createDimensions(double x, double y, double z) -> geometry_msgs::msg::Vector3
  {
    return geometry_msgs::build<geometry_msgs::msg::Vector3>().x(x).y(y).z(z);
  }

  auto initializeEntityStatus(
    traffic_simulator_msgs::EntityStatus & status, const std::string & name,
    traffic_simulator_msgs::EntityType::Enum type, const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Vector3 & dimensions) -> void
  {
    status.set_name(name);
    status.mutable_type()->set_type(type);

    auto new_pose = status.mutable_pose();
    auto new_position = new_pose->mutable_position();
    new_position->set_x(pose.position.x);
    new_position->set_y(pose.position.y);
    new_position->set_z(pose.position.z);

    auto new_orientation = new_pose->mutable_orientation();
    new_orientation->set_x(pose.orientation.x);
    new_orientation->set_y(pose.orientation.y);
    new_orientation->set_z(pose.orientation.z);
    new_orientation->set_w(pose.orientation.w);

    auto new_bounding_box = status.mutable_bounding_box();
    auto new_dimensions = new_bounding_box->mutable_dimensions();
    new_dimensions->set_x(dimensions.x);
    new_dimensions->set_y(dimensions.y);
    new_dimensions->set_z(dimensions.z);
  }

  auto createRosInterface() -> void
  {
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_output", 10);
    subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "lidar_output", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { received_msg_ = msg; });
  }
};
#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_LIDAR_SENSOR_HPP_
