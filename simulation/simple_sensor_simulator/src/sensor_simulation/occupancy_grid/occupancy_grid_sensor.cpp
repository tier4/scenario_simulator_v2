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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <simple_sensor_simulator/exception.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/occupancy_grid_generator.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/occupancy_grid_sensor.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
geometry_msgs::Pose OccupancyGridSensorBase::getSensorPose(
  const std::vector<traffic_simulator_msgs::EntityStatus> & status) const
{
  for (const auto & s : status) {
    if (
      s.type().type() == traffic_simulator_msgs::EntityType::EGO &&
      s.name() == configuration_.entity()) {
      return s.pose();
    }
  }
  throw SimulationRuntimeError("Occupancy grid sensor can be attached only ego entity.");
}

const std::vector<std::string> OccupancyGridSensorBase::getDetectedObjects(
  const std::vector<traffic_simulator_msgs::EntityStatus> & status) const
{
  std::vector<std::string> detected_objects;
  const auto pose = getSensorPose(status);
  for (const auto & s : status) {
    double distance = std::hypot(
      s.pose().position().x() - pose.position().x(), s.pose().position().y() - pose.position().y(),
      s.pose().position().z() - pose.position().z());
    if (s.name() != configuration_.entity() && distance <= configuration_.range()) {
      detected_objects.emplace_back(s.name());
    }
  }
  return detected_objects;
}

template <>
auto OccupancyGridSensor<nav_msgs::msg::OccupancyGrid>::getOccupancyGrid(
  const std::vector<traffic_simulator_msgs::EntityStatus> & status, const rclcpp::Time & stamp,
  const std::vector<std::string> & lidar_detected_entity) -> nav_msgs::msg::OccupancyGrid
{
  std::vector<std::string> detected_objects;
  if (configuration_.filter_by_range()) {
    detected_objects = getDetectedObjects(status);
  } else {
    detected_objects = lidar_detected_entity;
  }
  boost::optional<geometry_msgs::msg::Pose> ego_pose_north_up;
  OccupancyGridGenerator generator(configuration_);
  for (const auto & s : status) {
    if (configuration_.entity() == s.name()) {
      geometry_msgs::msg::Pose pose;
      simulation_interface::toMsg(s.pose(), pose);
      pose.orientation = geometry_msgs::msg::Quaternion();
      ego_pose_north_up = pose;
    }
  }
  for (const auto & s : status) {
    if (configuration_.entity() != s.name()) {
      auto result = std::find(detected_objects.begin(), detected_objects.end(), s.name());
      if (result == detected_objects.end()) {
        continue;
      }
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
      generator.addPrimitive<simple_sensor_simulator::primitives::Box>(
        s.name(), s.bounding_box().dimensions().x(), s.bounding_box().dimensions().y(),
        s.bounding_box().dimensions().z(), pose);
    }
  }
  if (ego_pose_north_up) {
    return generator.generate(ego_pose_north_up.get(), stamp);
  } else {
    throw SimulationRuntimeError("Failed to calculate ego pose with north up.");
  }
}
}  // namespace simple_sensor_simulator