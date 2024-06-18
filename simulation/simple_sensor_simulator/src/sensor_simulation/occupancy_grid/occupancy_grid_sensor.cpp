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

#include <algorithm>
#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
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
  const std::vector<traffic_simulator_msgs::EntityStatus> & status,
  const std::vector<std::string> & lidar_detected_entities) const
{
  std::vector<std::string> detected_entities;
  const auto pose = getSensorPose(status);
  for (const auto & s : status) {
    if (const auto has_detected =
          std::find(lidar_detected_entities.begin(), lidar_detected_entities.end(), s.name()) !=
          lidar_detected_entities.end();
        !has_detected) {
      continue;
    }

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
  const std::vector<std::string> & lidar_detected_entities) -> nav_msgs::msg::OccupancyGrid
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

  // find ego from `status` and get its pose with north side up
  auto ego_pose_north_up = geometry_msgs::msg::Pose();
  {
    auto is_ego = [&](const auto & s) { return configuration_.entity() == s.name(); };
    auto ego = std::find_if(status.begin(), status.end(), is_ego);
    if (ego == status.end()) {
      throw SimulationRuntimeError("Failed to calculate ego pose with north up.");
    }
    simulation_interface::toMsg(ego->pose(), ego_pose_north_up);
    /**
     * @note
     * There is no problem with the yaw axis being north-up, but unless the pitch and roll axes are
     * adjusted to the slope of the road surface, the grid map will not project the obstacles correctly.
     * However, the current implementation of autoware.universe does not consider the roll and pitch axes,
     * so it is not considered here either.
     */
    ego_pose_north_up.orientation = geometry_msgs::msg::Quaternion();
  }

  // construct a `set` of detected object names to look up entities later
  auto detected_entities = std::set<std::string>();
  {
    if (configuration_.filter_by_range()) {
      auto v = getDetectedObjects(status, lidar_detected_entities);
      detected_entities.insert(v.begin(), v.end());
    } else {
      detected_entities.insert(lidar_detected_entities.begin(), lidar_detected_entities.end());
    }
  }

  // construct an occupancy grid
  builder_.reset(ego_pose_north_up);
  for (const auto & s : status) {
    if (configuration_.entity() != s.name()) {
      // skip if entity is not actually detected
      if (detected_entities.count(s.name()) == 0) {
        continue;
      }

      auto pose = geometry_msgs::msg::Pose();
      {
        simulation_interface::toMsg(s.pose(), pose);
        auto point = geometry_msgs::msg::Point();
        simulation_interface::toMsg(s.bounding_box().center(), point);
        auto rotation = math::geometry::getRotationMatrix(pose.orientation);
        auto center = (rotation * Eigen::Vector3d(point.x, point.y, point.z)).eval();

        pose.position.x += center.x();
        pose.position.y += center.y();
        pose.position.z += center.z();
      }

      const auto & v = s.bounding_box().dimensions();
      builder_.add(primitives::Box(v.x(), v.y(), v.z(), pose));
    }
  }
  builder_.build();

  // construct message
  auto res = nav_msgs::msg::OccupancyGrid();
  res.header.stamp = stamp;
  res.header.frame_id = "map";
  res.data = builder_.get();
  res.info.height = configuration_.height();
  res.info.width = configuration_.width();
  res.info.map_load_time = stamp;
  res.info.resolution = configuration_.resolution();
  res.info.origin = ego_pose_north_up;
  res.info.origin.position.x -= 0.5 * configuration_.height() * configuration_.resolution();
  res.info.origin.position.y -= 0.5 * configuration_.width() * configuration_.resolution();
  return res;
}
}  // namespace simple_sensor_simulator
