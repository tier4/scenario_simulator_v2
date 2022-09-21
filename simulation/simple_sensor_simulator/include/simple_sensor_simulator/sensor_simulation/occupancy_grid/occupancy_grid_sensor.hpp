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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__OCCUPANCY_GRID_SENSOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__OCCUPANCY_GRID_SENSOR_HPP_


#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <simulation_api_schema.pb.h>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/occupancy_grid_builder.hpp>

namespace simple_sensor_simulator
{
/**
 * @brief Base class of occupancy grid sensor simulator
 */
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

  /**
   * @brief Update sensor status
   */
  virtual void update(
    const double, const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &,
    const std::vector<std::string> & lidar_detected_entity) = 0;

  /**
   * @brief List all objects in range of sensor sight
   * @return names of objects in range of sensor sight
   */
  const std::vector<std::string> getDetectedObjects(
    const std::vector<traffic_simulator_msgs::EntityStatus> & status) const;

  /**
   * @brief Extract sensor pose from entity statuses
   * @return sensor pose
   * @warning `status` must contain EGO object
   * @exception SimulationRuntimeError if `status` does not contain EGO object
   */
  geometry_msgs::Pose getSensorPose(
    const std::vector<traffic_simulator_msgs::EntityStatus> & status) const;
};

/**
 * @brief occupancy grid sensor implemtation
 */
template <typename T>
class OccupancyGridSensor : public OccupancyGridSensorBase
{
  const typename rclcpp::Publisher<T>::SharedPtr publisher_ptr_;

  /**
   * @brief construct occupancy grid from entity list
   * @return occupancy grid of specified type
   */
  auto getOccupancyGrid(
    const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &,
    const std::vector<std::string> &) -> T;

public:
  explicit OccupancyGridSensor(
    const double current_time,
    const simulation_api_schema::OccupancyGridSensorConfiguration & configuration,
    const typename rclcpp::Publisher<T>::SharedPtr & publisher_ptr)
  : OccupancyGridSensorBase(current_time, configuration),
    publisher_ptr_(publisher_ptr),
    builder_(configuration.resolution(), configuration.height(), configuration.width())
  {
  }

  auto update(
    const double current_time, const std::vector<traffic_simulator_msgs::EntityStatus> & status,
    const rclcpp::Time & stamp, const std::vector<std::string> & lidar_detected_entity)
    -> void override
  {
    if (current_time - last_update_stamp_ - configuration_.update_duration() >= -0.002) {
      last_update_stamp_ = current_time;
      publisher_ptr_->publish(getOccupancyGrid(status, stamp, lidar_detected_entity));
    } else {
      detected_objects_ = {};
    }
  }

private:
  mutable OccupancyGridBuilder builder_;
};

template <>
auto OccupancyGridSensor<nav_msgs::msg::OccupancyGrid>::getOccupancyGrid(
  const std::vector<traffic_simulator_msgs::EntityStatus> & status, const rclcpp::Time & stamp,
  const std::vector<std::string> & lidar_detected_entity) -> nav_msgs::msg::OccupancyGrid;
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__OCCUPANCY_GRID_SENSOR_HPP_
