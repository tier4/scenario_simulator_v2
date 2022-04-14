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

#include <quaternion_operation/quaternion_operation.h>

#include <boost/optional.hpp>
#include <memory>
#include <simple_sensor_simulator/exception.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
template <>
auto LidarSensor<sensor_msgs::msg::PointCloud2>::raycast(
  const std::vector<traffic_simulator_msgs::EntityStatus> & status, const rclcpp::Time & stamp)
  -> sensor_msgs::msg::PointCloud2
{
  Raycaster raycaster;
  boost::optional<geometry_msgs::msg::Pose> ego_pose;
  for (const auto & s : status) {
    if (configuration_.entity() == s.name()) {
      geometry_msgs::msg::Pose pose;
      simulation_interface::toMsg(s.pose(), pose);
      ego_pose = pose;
    } else {
      geometry_msgs::msg::Pose pose;
      simulation_interface::toMsg(s.pose(), pose);
      auto rotation = quaternion_operation::getRotationMatrix(pose.orientation);
      geometry_msgs::msg::Point center_point;
      simulation_interface::toMsg(s.bounding_box().center(), center_point);
      Eigen::Vector3d center(center_point.x, center_point.y, center_point.z);
      center = rotation * center;
      pose.position.x = pose.position.x + center.x();
      pose.position.y = pose.position.y + center.y();
      pose.position.z = pose.position.z + center.z();
      raycaster.addPrimitive<simple_sensor_simulator::primitives::Box>(
        s.name(), s.bounding_box().dimensions().x(), s.bounding_box().dimensions().y(),
        s.bounding_box().dimensions().z(), pose);
    }
  }
  if (ego_pose) {
    std::vector<double> vertical_angles;
    for (const auto v : configuration_.vertical_angles()) {
      vertical_angles.emplace_back(v);
    }
    const auto pointcloud = raycaster.raycast(
      "base_link", stamp, ego_pose.get(), configuration_.horizontal_resolution(), vertical_angles);
    detected_objects_ = raycaster.getDetectedObject();
    return pointcloud;
  }
  throw simple_sensor_simulator::SimulationRuntimeError("failed to found ego vehicle");
}
}  // namespace simple_sensor_simulator
