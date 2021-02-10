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

#include <scenario_simulator/sensor_simulation/sensor_simulation.hpp>

#include <memory>
#include <vector>

namespace scenario_simulator
{
SensorSimulation::SensorSimulation(std::shared_ptr<rclcpp::Clock> clock_ptr)
{
  clock_ptr_ = clock_ptr;
}

void SensorSimulation::attachLidarSensor(
  const simulation_api_schema::LidarConfiguration & configuration,
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> publisher_ptr)
{
  LidarModel lidar_model(configuration, publisher_ptr);
  lidar_models_.push_back(lidar_model);
}

void SensorSimulation::updateSensorFrame(
  double current_time,
  const std::vector<openscenario_msgs::EntityStatus> & status)
{
  const auto now = clock_ptr_->now();
  for (auto & model : lidar_models_) {
    model.update(current_time, status, now);
  }
}
}  // namespace scenario_simulator
