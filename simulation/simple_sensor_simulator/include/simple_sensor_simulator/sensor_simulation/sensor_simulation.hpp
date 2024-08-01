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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__SENSOR_SIMULATION_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__SENSOR_SIMULATION_HPP_

#include <simulation_api_schema.pb.h>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/detection_sensor/detection_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/imu/imu_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/occupancy_grid_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/traffic_lights/traffic_lights_detector.hpp>
#include <vector>

namespace simple_sensor_simulator
{
class SensorSimulation
{
public:
  auto attachLidarSensor(
    const double current_simulation_time,
    const simulation_api_schema::LidarConfiguration & configuration, rclcpp::Node & node) -> void
  {
    if (configuration.architecture_type().find("awf/universe") != std::string::npos) {
      lidar_sensors_.push_back(std::make_unique<LidarSensor<sensor_msgs::msg::PointCloud2>>(
        current_simulation_time, configuration,
        node.create_publisher<sensor_msgs::msg::PointCloud2>(
          "/perception/obstacle_segmentation/pointcloud", 1)));
    } else {
      std::stringstream ss;
      ss << "Unexpected architecture_type " << std::quoted(configuration.architecture_type())
         << " given.";
      throw std::runtime_error(ss.str());
    }
  }

  auto attachDetectionSensor(
    const double current_simulation_time,
    const simulation_api_schema::DetectionSensorConfiguration & configuration, rclcpp::Node & node)
    -> void
  {
    if (configuration.architecture_type().find("awf/universe") != std::string::npos) {
      using Message = autoware_auto_perception_msgs::msg::DetectedObjects;
      using GroundTruthMessage = autoware_auto_perception_msgs::msg::TrackedObjects;
      detection_sensors_.push_back(std::make_unique<DetectionSensor<Message>>(
        current_simulation_time, configuration,
        node.create_publisher<Message>("/perception/object_recognition/detection/objects", 1),
        node.create_publisher<GroundTruthMessage>(
          "/perception/object_recognition/ground_truth/objects", 1)));
    } else {
      std::stringstream ss;
      ss << "Unexpected architecture_type " << std::quoted(configuration.architecture_type())
         << " given.";
      throw std::runtime_error(ss.str());
    }
  }

  auto attachOccupancyGridSensor(
    const double current_simulation_time,
    const simulation_api_schema::OccupancyGridSensorConfiguration & configuration,
    rclcpp::Node & node) -> void
  {
    if (configuration.architecture_type().find("awf/universe") != std::string::npos) {
      using Message = nav_msgs::msg::OccupancyGrid;
      occupancy_grid_sensors_.push_back(std::make_unique<OccupancyGridSensor<Message>>(
        current_simulation_time, configuration,
        node.create_publisher<Message>("/perception/occupancy_grid_map/map", 1)));
    } else {
      std::stringstream ss;
      ss << "Unexpected architecture_type " << std::quoted(configuration.architecture_type())
         << " given.";
      throw std::runtime_error(ss.str());
    }
  }

  auto attachPseudoTrafficLightsDetector(
    const double /*current_simulation_time*/,
    const simulation_api_schema::PseudoTrafficLightDetectorConfiguration & configuration,
    rclcpp::Node & node, std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils) -> void
  {
    if (configuration.architecture_type() == "awf/universe") {
      using Message = autoware_auto_perception_msgs::msg::TrafficSignalArray;
      traffic_lights_detectors_.push_back(std::make_unique<traffic_lights::TrafficLightsDetector>(
        std::make_shared<traffic_simulator::TrafficLightPublisher<Message>>(
          "/perception/traffic_light_recognition/traffic_signals", &node, hdmap_utils)));
    } else if (configuration.architecture_type() >= "awf/universe/20230906") {
      using Message = autoware_perception_msgs::msg::TrafficSignalArray;
      traffic_lights_detectors_.push_back(std::make_unique<traffic_lights::TrafficLightsDetector>(
        std::make_shared<traffic_simulator::TrafficLightPublisher<Message>>(
          "/perception/traffic_light_recognition/internal/traffic_signals", &node, hdmap_utils)));
    } else {
      std::stringstream ss;
      ss << "Unexpected architecture_type " << std::quoted(configuration.architecture_type())
         << " given.";
      throw std::runtime_error(ss.str());
    }
  }

  auto attachImuSensor(
    const double /*current_simulation_time*/,
    const simulation_api_schema::ImuSensorConfiguration & configuration, rclcpp::Node & node)
    -> void
  {
    imu_sensors_.push_back(std::make_unique<ImuSensor<sensor_msgs::msg::Imu>>(
      configuration, node.create_publisher<sensor_msgs::msg::Imu>("/sensing/imu/imu_data", 1)));
  }

  auto updateSensorFrame(
    double current_simulation_time, const rclcpp::Time & current_ros_time,
    const std::vector<traffic_simulator_msgs::EntityStatus> &,
    const simulation_api_schema::UpdateTrafficLightsRequest &) -> void;

private:
  std::vector<std::unique_ptr<ImuSensorBase>> imu_sensors_;
  std::vector<std::unique_ptr<LidarSensorBase>> lidar_sensors_;
  std::vector<std::unique_ptr<DetectionSensorBase>> detection_sensors_;
  std::vector<std::unique_ptr<OccupancyGridSensorBase>> occupancy_grid_sensors_;
  std::vector<std::unique_ptr<traffic_lights::TrafficLightsDetector>> traffic_lights_detectors_;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__SENSOR_SIMULATION_HPP_
