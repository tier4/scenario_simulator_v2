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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__OCCUPANCY_GRID_SENSOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__OCCUPANCY_GRID_SENSOR_HPP_

#include <simulation_api_schema.pb.h>

#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
class OccupancyGridSensorBase
{
protected:
  double last_update_stamp_;

  simulation_api_schema::OccupancyGridSensorConfiguration configuration_;

  std::vector<std::string> detected_objects_;

  explicit OccupancyGridSensorBase(
    const double last_update_stamp,
    const simulation_api_schema::OccupancyGridSensorConfiguration & configuration)
  : last_update_stamp_(last_update_stamp), configuration_(configuration)
  {
  }

public:
  virtual ~OccupancyGridSensorBase() = default;

  virtual auto update(
    const double, const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &)
    -> void = 0;

  auto getDetectedObjects() const -> const std::vector<std::string> & { return detected_objects_; }
};

template <typename T>
class OccupancyGridSensor : public OccupancyGridSensorBase
{
  const typename rclcpp::Publisher<T>::SharedPtr publisher_ptr_;

  auto getOccupancyGrid(
    const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &) -> T;

public:
  explicit OccupancyGridSensor(
    const double current_time,
    const simulation_api_schema::OccupancyGridSensorConfiguration & configuration,
    const typename rclcpp::Publisher<T>::SharedPtr & publisher_ptr)
  : OccupancyGridSensorBase(current_time, configuration), publisher_ptr_(publisher_ptr)
  {
  }

  auto update(
    const double current_time, const std::vector<traffic_simulator_msgs::EntityStatus> & status,
    const rclcpp::Time & stamp) -> void override
  {
    if (current_time - last_update_stamp_ - configuration_.update_duration() >= -0.002) {
      last_update_stamp_ = current_time;
      publisher_ptr_->publish(getOccupancyGrid(status, stamp));
    } else {
      detected_objects_ = {};
    }
  }
};

template <>
auto OccupancyGridSensor<nav_msgs::msg::OccupancyGrid>::getOccupancyGrid(
  const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &)
  -> nav_msgs::msg::OccupancyGrid;
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__OCCUPANCY_GRID_SENSOR_HPP_