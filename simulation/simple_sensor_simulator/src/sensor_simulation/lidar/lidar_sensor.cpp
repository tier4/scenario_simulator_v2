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
  const rclcpp::Time & current_ros_time) -> sensor_msgs::msg::PointCloud2
{
  std::optional<geometry_msgs::msg::Pose> ego_pose;

  for (const auto & entity : entities) {
    if (configuration_.entity() == entity.name()) {
      geometry_msgs::msg::Pose pose;
      simulation_interface::toMsg(entity.pose(), pose);
      ego_pose = pose;
    } else {
      geometry_msgs::msg::Pose pose;
      simulation_interface::toMsg(entity.pose(), pose);
      auto rotation = math::geometry::getRotationMatrix(pose.orientation);
      geometry_msgs::msg::Point center_point;
      simulation_interface::toMsg(entity.bounding_box().center(), center_point);
      Eigen::Vector3d center(center_point.x, center_point.y, center_point.z);
      center = rotation * center;
      pose.position.x = pose.position.x + center.x();
      pose.position.y = pose.position.y + center.y();
      pose.position.z = pose.position.z + center.z();
      raycaster_.addPrimitive<simple_sensor_simulator::primitives::Box>(
        entity.name(),                           //
        entity.bounding_box().dimensions().x(),  //
        entity.bounding_box().dimensions().y(),  //
        entity.bounding_box().dimensions().z(),  //
        pose);
    }
  }

  if (ego_pose) {
    std::vector<double> vertical_angles;
    for (const auto vertical_angle : configuration_.vertical_angles()) {
      vertical_angles.push_back(vertical_angle);
    }
    const auto pointcloud = raycaster_.raycast("base_link", current_ros_time, ego_pose.value());
    detected_objects_ = raycaster_.getDetectedObject();
    return pointcloud;
  } else {
    throw simple_sensor_simulator::SimulationRuntimeError("failed to find ego vehicle");
  }
}
}  // namespace simple_sensor_simulator
