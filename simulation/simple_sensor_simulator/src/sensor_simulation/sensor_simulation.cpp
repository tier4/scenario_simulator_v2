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
auto SensorSimulation::updateSensorFrame(
  double current_simulation_time, const rclcpp::Time & current_ros_time,
  const std::vector<traffic_simulator_msgs::EntityStatus> & entities,
  const simulation_api_schema::UpdateTrafficLightsRequest & update_traffic_lights_request) -> void
{
  for (auto & sensor : imu_sensors_) {
    sensor->update(current_ros_time, entities);
  }

  std::vector<std::string> lidar_detected_objects = {};
  for (auto & sensor : lidar_sensors_) {
    sensor->update(current_simulation_time, entities, current_ros_time);
    for (const auto & object : sensor->getDetectedObjects()) {
      if (std::count(lidar_detected_objects.begin(), lidar_detected_objects.end(), object) == 0) {
        lidar_detected_objects.push_back(object);
      }
    }
  }

  for (auto & sensor : detection_sensors_) {
    sensor->update(current_simulation_time, entities, current_ros_time, lidar_detected_objects);
  }

  for (auto & sensor : occupancy_grid_sensors_) {
    sensor->update(current_simulation_time, entities, current_ros_time, lidar_detected_objects);
  }

  for (auto & sensor : traffic_lights_detectors_) {
    sensor->updateFrame(current_ros_time, update_traffic_lights_request);
  }
}
}  // namespace simple_sensor_simulator
