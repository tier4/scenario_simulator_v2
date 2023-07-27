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

#include <memory>
#include <simple_sensor_simulator/sensor_simulation/sensor_simulation.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
void SensorSimulation::updateSensorFrame(
  double current_time, const rclcpp::Time & current_ros_time,
  const std::vector<traffic_simulator_msgs::EntityStatus> & status,
  const std::vector<autoware_auto_perception_msgs::msg::TrafficSignal> & traffic_signals)
{
  std::vector<std::string> lidar_detected_objects = {};
  for (auto & sensor : lidar_sensors_) {
    sensor->update(current_time, status, current_ros_time);
    const auto objects = sensor->getDetectedObjects();
    for (const auto & obj : objects) {
      if (std::count(lidar_detected_objects.begin(), lidar_detected_objects.end(), obj) == 0) {
        lidar_detected_objects.push_back(obj);
      }
    }
  }
  for (auto & sensor : detection_sensors_) {
    sensor->update(current_time, status, current_ros_time, lidar_detected_objects);
  }
  for (auto & sensor : occupancy_grid_sensors_) {
    sensor->update(current_time, status, current_ros_time, lidar_detected_objects);
  }

  for (auto & sensor : traffic_lights_detectors_) {
    sensor->updateFrame(current_ros_time, traffic_signals);
  }
}
}  // namespace simple_sensor_simulator
