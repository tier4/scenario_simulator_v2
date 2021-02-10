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

#ifndef SCENARIO_SIMULATOR__SENSOR_SIMULATION__SENSOR_SIMULATION_HPP_
#define SCENARIO_SIMULATOR__SENSOR_SIMULATION__SENSOR_SIMULATION_HPP_

#include <scenario_simulator/sensor_simulation/lidar/lidar_model.hpp>

#include <simulation_api_schema.pb.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace scenario_simulator
{
class SensorSimulation
{
public:
  SensorSimulation(std::shared_ptr<rclcpp::Clock> clock_ptr);
  void attachLidarSensor(
    const simulation_api_schema::LidarConfiguration & configuration,
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> publisher_ptr);
  void updateSensorFrame(
    double current_time,
    const std::vector<openscenario_msgs::EntityStatus> & status);

private:
  std::vector<LidarModel> lidar_models_;
  std::shared_ptr<rclcpp::Clock> clock_ptr_;
};
}  // namespace scenario_simulator

#endif  // SCENARIO_SIMULATOR__SENSOR_SIMULATION__SENSOR_SIMULATION_HPP_
