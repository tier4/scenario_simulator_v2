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

#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace simple_sensor_simulator
{
Raycaster::Entity::Entity(const traffic_simulator_msgs::EntityStatus & status)
: entity_status(status)
{
  geometry_msgs::msg::Pose pose;
  simulation_interface::toMsg(entity_status.pose(), pose);
  auto rotation = math::geometry::getRotationMatrix(pose.orientation);
  geometry_msgs::msg::Point center_point;
  simulation_interface::toMsg(entity_status.bounding_box().center(), center_point);
  Eigen::Vector3d center(center_point.x, center_point.y, center_point.z);
  center = rotation * center;
  pose.position.x = pose.position.x + center.x();
  pose.position.y = pose.position.y + center.y();
  pose.position.z = pose.position.z + center.z();

  primitive = std::make_unique<simple_sensor_simulator::primitives::Box>(
    entity_status.bounding_box().dimensions().x(), entity_status.bounding_box().dimensions().y(),
    entity_status.bounding_box().dimensions().z(), pose);
}

Raycaster::Raycaster()
: device_(rtcNewDevice(nullptr)), scene_(rtcNewScene(device_)), engine_(seed_gen_())
{
}

Raycaster::Raycaster(std::string embree_config)
: device_(rtcNewDevice(embree_config.c_str())), scene_(rtcNewScene(device_)), engine_(seed_gen_())
{
}

Raycaster::~Raycaster()
{
  rtcReleaseScene(scene_);
  rtcReleaseDevice(device_);
}

void Raycaster::setDirection(
  const simulation_api_schema::LidarConfiguration & configuration, double horizontal_angle_start,
  double horizontal_angle_end)
{
  std::vector<double> vertical_angles;
  for (const auto v : configuration.vertical_angles()) {
    vertical_angles.emplace_back(v);
  }

  auto quat_directions = getDirections(
    vertical_angles, horizontal_angle_start, horizontal_angle_end,
    configuration.horizontal_resolution());
  rotation_matrices_.clear();
  for (const auto & q : quat_directions) {
    rotation_matrices_.push_back(math::geometry::getRotationMatrix(q));
  }
}

std::vector<geometry_msgs::msg::Quaternion> Raycaster::getDirections(
  const std::vector<double> & vertical_angles, double horizontal_angle_start,
  double horizontal_angle_end, double horizontal_resolution)
{
  if (
    directions_.empty() || previous_horizontal_angle_start_ != horizontal_angle_start ||
    previous_horizontal_angle_end_ != horizontal_angle_end ||
    previous_horizontal_resolution_ != horizontal_resolution ||
    previous_vertical_angles_ != vertical_angles) {
    std::vector<geometry_msgs::msg::Quaternion> directions;
    double horizontal_angle = horizontal_angle_start;
    while (horizontal_angle <= horizontal_angle_end) {
      horizontal_angle = horizontal_angle + horizontal_resolution;
      for (const auto vertical_angle : vertical_angles) {
        geometry_msgs::msg::Vector3 rpy;
        rpy.x = 0;
        rpy.y = vertical_angle;
        rpy.z = horizontal_angle;
        auto quat = math::geometry::convertEulerAngleToQuaternion(rpy);
        directions.emplace_back(quat);
      }
    }
    directions_ = directions;
    previous_horizontal_angle_end_ = horizontal_angle_end;
    previous_horizontal_angle_start_ = horizontal_angle_start;
    previous_horizontal_resolution_ = horizontal_resolution;
    previous_vertical_angles_ = vertical_angles;
  }
  return directions_;
}

Raycaster::RaycastResult Raycaster::raycast(
  const geometry_msgs::msg::Pose & origin, std::vector<Entity> & entities, double max_distance,
  double min_distance)
{
  RaycastResult result(entities);

  std::unordered_map<uint32_t, size_t> geometry_id_to_entity_index;
  for (size_t entity_idx = 0; entity_idx < entities.size(); ++entity_idx) {
    auto & entity = entities[entity_idx];
    entity.geometry_id = entity.primitive->addToScene(device_, scene_);
    geometry_id_to_entity_index[entity.geometry_id.value()] = entity_idx;
  }

  std::vector<uint32_t> point_geometry_ids;

  rtcCommitScene(scene_);
  intersect(result.cloud, origin, point_geometry_ids, max_distance, min_distance);

  result.point_to_entity_index.reserve(point_geometry_ids.size());
  for (const auto & geometry_id : point_geometry_ids) {
    auto it = geometry_id_to_entity_index.find(geometry_id);
    if (it != geometry_id_to_entity_index.end()) {
      result.point_to_entity_index.push_back(it->second);
    }
  }

  for (auto & entity : entities) {
    if (entity.geometry_id.has_value()) {
      rtcDetachGeometry(scene_, entity.geometry_id.value());
    }
  }

  return result;
}
}  // namespace simple_sensor_simulator
