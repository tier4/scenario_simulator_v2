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
  template <typename... Ts>
  void attachLidarSensor(Ts &&... xs)
  {
    lidar_sensors_.push_back(std::make_unique<LidarSensor<sensor_msgs::msg::PointCloud2>>(
      std::forward<decltype(xs)>(xs)...));
  }

  template <typename... Ts>
  void attachDetectionSensor(Ts &&... xs)
  {
    detection_sensors_.push_back(
      std::make_unique<DetectionSensor<autoware_auto_perception_msgs::msg::PredictedObjects>>(
        std::forward<decltype(xs)>(xs)...));
  }

  void updateSensorFrame(
    double current_time, const rclcpp::Time & current_ros_time,
    const std::vector<traffic_simulator_msgs::EntityStatus> & status);

private:
  std::vector<std::unique_ptr<LiDARSensorBase>> lidar_sensors_;
  std::vector<std::unique_ptr<DetectionSensorBase>> detection_sensors_;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__SENSOR_SIMULATION_HPP_
