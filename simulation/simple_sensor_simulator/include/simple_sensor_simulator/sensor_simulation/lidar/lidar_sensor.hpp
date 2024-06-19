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

#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
class LidarSensorBase
{
protected:
  double previous_simulation_time_;

  simulation_api_schema::LidarConfiguration configuration_;

  Raycaster raycaster_;
  std::vector<std::string> detected_objects_;

  explicit LidarSensorBase(
    const double current_simulation_time,
    const simulation_api_schema::LidarConfiguration & configuration)
  : previous_simulation_time_(current_simulation_time), configuration_(configuration)
  {
  }

public:
  virtual ~LidarSensorBase() = default;

  virtual auto update(
    const double current_simulation_time, const std::vector<traffic_simulator_msgs::EntityStatus> &,
    const rclcpp::Time & current_ros_time) -> void = 0;

  auto getDetectedObjects() const -> const std::vector<std::string> & { return detected_objects_; }
};

template <typename T>
class LidarSensor : public LidarSensorBase
{
  const typename rclcpp::Publisher<T>::SharedPtr publisher_ptr_;

  std::queue<std::pair<sensor_msgs::msg::PointCloud2, double>> queue_pointcloud_;

  auto raycast(const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &)
    -> T;

public:
  explicit LidarSensor(
    const double current_simulation_time,
    const simulation_api_schema::LidarConfiguration & configuration,
    const typename rclcpp::Publisher<T>::SharedPtr & publisher_ptr)
  : LidarSensorBase(current_simulation_time, configuration), publisher_ptr_(publisher_ptr)
  {
    raycaster_.setDirection(configuration);
  }

  auto update(
    const double current_simulation_time,
    const std::vector<traffic_simulator_msgs::EntityStatus> & status,
    const rclcpp::Time & current_ros_time) -> void override
  {
    if (
      current_simulation_time - previous_simulation_time_ - configuration_.scan_duration() >=
      -0.002) {
      previous_simulation_time_ = current_simulation_time;
      queue_pointcloud_.push(
        std::make_pair(raycast(status, current_ros_time), current_simulation_time));
    } else {
      detected_objects_.clear();
    }

    if (
      not queue_pointcloud_.empty() and
      current_simulation_time - queue_pointcloud_.front().second >=
        configuration_.lidar_sensor_delay()) {
      const auto pointcloud = queue_pointcloud_.front().first;
      queue_pointcloud_.pop();
      publisher_ptr_->publish(pointcloud);
    }
  }

private:
  // Override
  Raycaster raycaster_;
};

template <>
auto LidarSensor<sensor_msgs::msg::PointCloud2>::raycast(
  const std::vector<traffic_simulator_msgs::EntityStatus> &, const rclcpp::Time &)
  -> sensor_msgs::msg::PointCloud2;
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__LIDAR_SENSOR_HPP_
