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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__OCCUPANCY_GRID_GENERATOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__OCCUPANCY_GRID_GENERATOR_HPP_

#include <simulation_api_schema.pb.h>

#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
class OccupancyGridGenerator
{
public:
  OccupancyGridGenerator(
    const simulation_api_schema::OccupancyGridSensorConfiguration & configuration);
  nav_msgs::msg::OccupancyGrid generate(
    const geometry_msgs::msg::Pose & ego_pose, const rclcpp::Time & stamp) const;
  template <typename T, typename... Ts>
  void addPrimitive(std::string name, Ts &&... xs)
  {
    if (primitive_ptrs_.count(name) != 0) {
      throw std::runtime_error("primitive " + name + " already exist.");
    }
    auto primitive_ptr = std::make_unique<T>(std::forward<Ts>(xs)...);
    primitive_ptrs_.emplace(name, std::move(primitive_ptr));
  }
  const simulation_api_schema::OccupancyGridSensorConfiguration configuration;

private:
  std::unordered_map<std::string, std::unique_ptr<primitives::Primitive>> primitive_ptrs_;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__OCCUPANCY_GRID_GENERATOR_HPP_