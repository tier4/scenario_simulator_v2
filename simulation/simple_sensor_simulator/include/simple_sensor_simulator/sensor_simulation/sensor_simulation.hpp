// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/detection_sensor/detection_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <vector>

namespace simple_sensor_simulator
{
class SensorSimulation
{
public:
  void attachLidarSensor(
    const double current_time, const simulation_api_schema::LidarConfiguration & configuration,
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> publisher_ptr);
  void attachDetectionSensor(
    const double current_time,
    const simulation_api_schema::DetectionSensorConfiguration & configuration,
    std::shared_ptr<rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>>
      publisher_ptr);
  void updateSensorFrame(
    double current_time, const rclcpp::Time & current_ros_time,
    const std::vector<traffic_simulator_msgs::EntityStatus> & status);

private:
  std::vector<LidarSensor> lidar_sensors_;
  std::vector<DetectionSensor> detection_sensors_;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__SENSOR_SIMULATION_HPP_
