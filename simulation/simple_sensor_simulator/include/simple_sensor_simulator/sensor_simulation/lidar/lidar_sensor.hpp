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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_SENSOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_SENSOR_HPP_

#include <simulation_api_schema.pb.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include <simple_sensor_simulator/sensor_simulation/measure_time.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>


namespace simple_sensor_simulator
{
class LidarSensorBase
{
protected:
  MeasureTime timer;
  double last_update_stamp_;
  Raycaster raycaster_;
  simulation_api_schema::LidarConfiguration configuration_;

  std::vector<std::string> detected_objects_;

  explicit LidarSensorBase(
    const double last_update_stamp, const simulation_api_schema::LidarConfiguration & configuration)
  : last_update_stamp_(last_update_stamp), configuration_(configuration)
  {
  }

public:
  virtual ~LidarSensorBase() = default;

  virtual auto update(
    const double, const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &)
    -> void = 0;

  auto getDetectedObjects() const -> const std::vector<std::string> & { return detected_objects_; }
};

template <typename T>
class LidarSensor : public LidarSensorBase
{
  const typename rclcpp::Publisher<T>::SharedPtr publisher_ptr_;

  auto raycast(const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &)
    -> T;

public:
  explicit LidarSensor(
    const double current_time, const simulation_api_schema::LidarConfiguration & configuration,
    const typename rclcpp::Publisher<T>::SharedPtr & publisher_ptr)
  : LidarSensorBase(current_time, configuration), publisher_ptr_(publisher_ptr)
  {
  }

  auto update(
    const double current_time, const std::vector<traffic_simulator_msgs::EntityStatus> & status,
    const rclcpp::Time & stamp) -> void override
  {
    if (current_time - last_update_stamp_ - configuration_.scan_duration() >= -0.002) {
      last_update_stamp_ = current_time;
      publisher_ptr_->publish(raycast(status, stamp));
    } else {
      detected_objects_ = {};
    }
  }
};

template <>
auto LidarSensor<sensor_msgs::msg::PointCloud2>::raycast(
  const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &)
  -> sensor_msgs::msg::PointCloud2;
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_SENSOR_HPP_
