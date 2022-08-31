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

#include <quaternion_operation/quaternion_operation.h>

#include <boost/optional.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <simple_sensor_simulator/exception.hpp>
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
  std::vector<std::string> detected_entities;
  const auto pose = getSensorPose(status);
  for (const auto & s : status) {
    double distance = std::hypot(
      s.pose().position().x() - pose.position().x(), s.pose().position().y() - pose.position().y(),
      s.pose().position().z() - pose.position().z());
    if (s.name() != configuration_.entity() && distance <= configuration_.range()) {
      detected_entities.emplace_back(s.name());
    }
  }
  return detected_entities;
}

template <>
auto OccupancyGridSensor<nav_msgs::msg::OccupancyGrid>::getOccupancyGrid(
  const std::vector<traffic_simulator_msgs::EntityStatus> & status, const rclcpp::Time & stamp,
  const std::vector<std::string> & lidar_detected_entity) -> nav_msgs::msg::OccupancyGrid
{
  // check if entities in `status` have unique names
  {
    auto unique_entities = std::set<std::string>();
    for (const auto & s : status) {
      if (configuration_.entity() != s.name()) {
        if (unique_entities.count(s.name()) != 0) {
          throw std::runtime_error(
            "status contains primitives with the same name: `" + s.name() + "`");
        }
        unique_entities.insert(s.name());
      }
    }
  }

  // find ego from `status` and get its pose with northside up
  auto ego_pose_north_up = geometry_msgs::msg::Pose();
  {
    auto is_ego = [&](const auto & s) { return configuration_.entity() == s.name(); };
    auto ego = std::find_if(status.begin(), status.end(), is_ego);
    if (ego == status.end()) {
      throw SimulationRuntimeError("Failed to calculate ego pose with north up.");
    }
    simulation_interface::toMsg(ego->pose(), ego_pose_north_up);
    ego_pose_north_up.orientation = geometry_msgs::msg::Quaternion();
  }

  // construct a `set` of detected object names to look up entities later
  auto detected_entities = std::set<std::string>();
  {
    auto v = configuration_.filter_by_range() ? getDetectedObjects(status) : lidar_detected_entity;
    detected_entities.insert(v.begin(), v.end());
  }

  // enumerate all primitive shapes of entities
  auto primitives = std::vector<std::unique_ptr<primitives::Primitive>>();
  for (const auto & s : status) {
    if (configuration_.entity() != s.name()) {
      // skip if entity is not actually detected
      if (detected_entities.count(s.name()) == 0) {
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

      primitives.push_back(std::make_unique<primitives::Box>(
        s.bounding_box().dimensions().x(), s.bounding_box().dimensions().y(),
        s.bounding_box().dimensions().z(), pose));
    }
  }

  // reset `Grid` and draw all primitive shapes on `Grid`
  grid_.reset(ego_pose_north_up);
  for (const auto & primitive : primitives) {
    grid_.addPrimitive(primitive);
  }

  // construct `OccupancyGrid`
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header.stamp = stamp;
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.data = grid_.getData();
  occupancy_grid.info.height = configuration_.height();
  occupancy_grid.info.width = configuration_.width();
  occupancy_grid.info.map_load_time = stamp;
  occupancy_grid.info.resolution = configuration_.resolution();
  occupancy_grid.info.origin = ego_pose_north_up;
  occupancy_grid.info.origin.position.x -=
    0.5 * configuration_.height() * configuration_.resolution();
  occupancy_grid.info.origin.position.y -=
    0.5 * configuration_.width() * configuration_.resolution();
  return occupancy_grid;
}
}  // namespace simple_sensor_simulator
