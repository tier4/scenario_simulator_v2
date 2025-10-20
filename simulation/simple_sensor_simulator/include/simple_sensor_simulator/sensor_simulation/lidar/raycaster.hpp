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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_

#include <embree4/rtcore.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <optional>
#include <random>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>
#include <string>
#include <utility>
#include <vector>

namespace simple_sensor_simulator
{
class Raycaster
{
public:
  Raycaster();
  explicit Raycaster(std::string embree_config);
  ~Raycaster();

  struct Entity
  {
    const traffic_simulator_msgs::EntityStatus& entity_status;
    std::unique_ptr<primitives::Primitive> primitive;
    std::optional<uint32_t> geometry_id;

    explicit Entity(const traffic_simulator_msgs::EntityStatus& status);

    const std::string& name() const { return entity_status.name(); }
  };

  struct RaycastResult
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    std::vector<size_t> point_to_entity_index;
    const std::vector<Entity>& raycast_entities;

    explicit RaycastResult(const std::vector<Entity>& entities)
    : cloud(new pcl::PointCloud<pcl::PointXYZI>), raycast_entities(entities)
    {
    }

    std::set<std::string> getDetectedEntityNames() const
    {
      std::set<std::string> detected_entity_names;
      for (const auto & entity_idx : point_to_entity_index) {
        detected_entity_names.insert(raycast_entities[entity_idx].name());
      }
      return detected_entity_names;
    }
  };

  RaycastResult raycast(
    const geometry_msgs::msg::Pose & origin, std::vector<Entity> & entities,
    double max_distance = 300, double min_distance = 0);
  void setDirection(
    const simulation_api_schema::LidarConfiguration & configuration,
    double horizontal_angle_start = 0, double horizontal_angle_end = 2 * M_PI);

private:
  std::vector<geometry_msgs::msg::Quaternion> getDirections(
    const std::vector<double> & vertical_angles, double horizontal_angle_start,
    double horizontal_angle_end, double horizontal_resolution);
  std::vector<geometry_msgs::msg::Quaternion> directions_;
  double previous_horizontal_angle_start_;
  double previous_horizontal_angle_end_;
  double previous_horizontal_resolution_;
  std::vector<double> previous_vertical_angles_;
  RTCDevice device_;
  RTCScene scene_;
  std::random_device seed_gen_;
  std::default_random_engine engine_;
  std::vector<Eigen::Matrix3d> rotation_matrices_;

  void intersect(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const geometry_msgs::msg::Pose & origin,
    std::vector<uint32_t> & point_geometry_ids, double max_distance, double min_distance)
  {
    const auto orientation_matrix = math::geometry::getRotationMatrix(origin.orientation);
    for (unsigned int i = 0; i < rotation_matrices_.size(); ++i) {
      RTCRayHit rayhit = {};
      rayhit.ray.org_x = origin.position.x;
      rayhit.ray.org_y = origin.position.y;
      rayhit.ray.org_z = origin.position.z;
      // make raycast interact with all objects
      rayhit.ray.mask = 0b11111111'11111111'11111111'11111111;
      rayhit.ray.tfar = max_distance;
      rayhit.ray.tnear = min_distance;
      rayhit.ray.flags = false;

      const auto & ray_direction = rotation_matrices_.at(i);
      const auto rotation_mat = orientation_matrix * ray_direction;
      rayhit.ray.dir_x = rotation_mat(0);
      rayhit.ray.dir_y = rotation_mat(1);
      rayhit.ray.dir_z = rotation_mat(2);
      rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
      rtcIntersect1(scene_, &rayhit);

      if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        double distance = rayhit.ray.tfar;
        pcl::PointXYZI p;
        {
          p.x = ray_direction(0) * distance;
          p.y = ray_direction(1) * distance;
          p.z = ray_direction(2) * distance;
        }
        cloud->emplace_back(p);
        point_geometry_ids.push_back(rayhit.hit.geomID);
      }
    }
  }
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__RAYCASTER_HPP_
