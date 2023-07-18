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

#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/detection_sensor/detection_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/occupancy_grid_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/traffic_lights/traffic_lights_detector.hpp>
#include <vector>

namespace simple_sensor_simulator
{
class SensorSimulation
{
public:
  explicit SensorSimulation(rclcpp::Node & node)
  {
    traffic_lights_detectors_.emplace_back(std::make_unique<traffic_lights::TrafficLightsDetector>(
      "/perception/traffic_light_recognition/traffic_signals", node));
  }

  auto attachLidarSensor(
    const double current_simulation_time,
    const simulation_api_schema::LidarConfiguration & configuration, rclcpp::Node & node) -> void
  {
    if (configuration.architecture_type() == "awf/universe") {
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
    if (configuration.architecture_type() == "awf/universe") {
      using Message = autoware_auto_perception_msgs::msg::DetectedObjects;
      detection_sensors_.push_back(std::make_unique<DetectionSensor<Message>>(
        current_simulation_time, configuration,
        node.create_publisher<Message>("/perception/object_recognition/detection/objects", 1)));
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
    if (configuration.architecture_type() == "awf/universe") {
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

  void updateSensorFrame(
    double current_time, const rclcpp::Time & current_ros_time,
    const std::vector<traffic_simulator_msgs::EntityStatus> & status,
    const std::vector<autoware_auto_perception_msgs::msg::TrafficSignal> & traffic_signals);

private:
  std::vector<std::unique_ptr<LidarSensorBase>> lidar_sensors_;
  std::vector<std::unique_ptr<DetectionSensorBase>> detection_sensors_;
  std::vector<std::unique_ptr<OccupancyGridSensorBase>> occupancy_grid_sensors_;
  std::vector<std::unique_ptr<traffic_lights::TrafficLightsDetector>> traffic_lights_detectors_;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__SENSOR_SIMULATION_HPP_
