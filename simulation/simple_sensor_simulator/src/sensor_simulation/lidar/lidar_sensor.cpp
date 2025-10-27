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
#include <optional>
#include <simple_sensor_simulator/exception.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
template <>
auto LidarSensor<sensor_msgs::msg::PointCloud2>::raycast(
  const std::vector<traffic_simulator_msgs::EntityStatus> & entities,
  const rclcpp::Time & current_ros_time, double current_simulation_time)
  -> sensor_msgs::msg::PointCloud2
{
  std::optional<geometry_msgs::msg::Pose> ego_pose;
  std::vector<Raycaster::Entity> raycast_entities;
  raycast_entities.reserve(entities.size());

  for (const auto & entity : entities) {
    if (configuration_.entity() == entity.name()) {
      geometry_msgs::msg::Pose pose;
      simulation_interface::toMsg(entity.pose(), pose);
      ego_pose = pose;
    } else {
      raycast_entities.emplace_back(entity);
    }
  }

  if (ego_pose) {
    // Use geometry-aware raycast to get points with entity association
    auto raycast_measurement =
      performance_monitor_.startMeasurement(LidarPerformanceMonitor::MetricType::RAYCAST);
    auto result = raycaster_.raycast(ego_pose.value(), raycast_entities);
    detected_objects_ = result.getDetectedEntityNames();

    // Apply noise if noise processor is available
    if (noise_processor_) {
      auto noise_measurement =
        performance_monitor_.startMeasurement(LidarPerformanceMonitor::MetricType::NOISE);

      noise_processor_->applyNoise(result, current_simulation_time, ego_pose.value());
    }

    // Convert to standard PointCloud2 message
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*(result.cloud), pointcloud_msg);
    pointcloud_msg.header.frame_id = "base_link";
    pointcloud_msg.header.stamp = current_ros_time;
    return pointcloud_msg;
  } else {
    throw simple_sensor_simulator::SimulationRuntimeError("failed to find ego vehicle");
  }
}
}  // namespace simple_sensor_simulator
