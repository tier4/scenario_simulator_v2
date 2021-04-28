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

#include <memory>
#include <simple_sensor_simulator/sensor_simulation/sensor_simulation.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
SensorSimulation::SensorSimulation(const std::shared_ptr<rclcpp::Clock> & clock_ptr)
: clock_ptr_(clock_ptr)
{
}

void SensorSimulation::attachLidarSensor(
  const double current_time, const simulation_api_schema::LidarConfiguration & configuration,
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> publisher_ptr)
{
  LidarSensor lidar_sensor(current_time, configuration, publisher_ptr);
  lidar_sensors_.push_back(lidar_sensor);
}

void SensorSimulation::attachDetectionSensor(
  const double current_time,
  const simulation_api_schema::DetectionSensorConfiguration & configuration,
  std::shared_ptr<rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectArray>>
    publisher_ptr)
{
  DetectionSensor detection_sensor(current_time, configuration, publisher_ptr);
  detection_sensors_.push_back(detection_sensor);
}

void SensorSimulation::updateSensorFrame(
  double current_time, const std::vector<openscenario_msgs::EntityStatus> & status)
{
  std::vector<std::string> detected_objects = {};
  const auto now = clock_ptr_->now();
  for (auto & sensor : lidar_sensors_) {
    sensor.update(current_time, status, now);
    const auto objects = sensor.getDetectedObjects();
    for (const auto & obj : objects) {
      if (std::count(detected_objects.begin(), detected_objects.end(), obj) == 0) {
        detected_objects.emplace_back(obj);
      }
    }
  }
  for (auto & sensor : detection_sensors_) {
    sensor.update(current_time, status, now, detected_objects);
  }
}
}  // namespace simple_sensor_simulator
