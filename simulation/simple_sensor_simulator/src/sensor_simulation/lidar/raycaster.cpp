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

#include <algorithm>
#include <iostream>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <iostream>

namespace simple_sensor_simulator
{
Raycaster::Raycaster() : primitive_ptrs_(0), device_(nullptr), scene_(nullptr), engine_(seed_gen_())
{
  device_ = rtcNewDevice(nullptr);
  scene_ = rtcNewScene(device_);
}

Raycaster::Raycaster(std::string embree_config)
: primitive_ptrs_(0), device_(nullptr), scene_(nullptr), engine_(seed_gen_())
{
  device_ = rtcNewDevice(embree_config.c_str());
  scene_ = rtcNewScene(device_);
}

Raycaster::~Raycaster()
{
  rtcReleaseScene(scene_);
  rtcReleaseDevice(device_);
}

std::vector<geometry_msgs::msg::Quaternion> Raycaster::getDirections(
    std::vector<double> vertical_angles, double horizontal_angle_start,
    double horizontal_angle_end, double horizontal_resolution)
  {
    // std::cerr << "directions size: " << directions_.size() << std::endl;
    // std::cerr << "start: " << previous_horizontal_angle_start_ << std::endl;
    // std::cerr << "end: " << previous_horizontal_angle_end_  << std::endl;
    // std::cerr << "res: " << previous_horizontal_resolution_  << std::endl;
    // std::cerr << "ver size: " << previous_vertical_angles_.size() << std::endl;
    if (directions_.empty() || previous_horizontal_angle_start_ != horizontal_angle_start ||
        previous_horizontal_angle_end_ != horizontal_angle_end || previous_horizontal_resolution_ != horizontal_resolution ||
        previous_vertical_angles_ != vertical_angles)
    {
      std::cerr << "Not matching" << std::endl;
      std::vector<geometry_msgs::msg::Quaternion> directions;
      double horizontal_angle = horizontal_angle_start;
      while (horizontal_angle <= (horizontal_angle_end)) {
        horizontal_angle = horizontal_angle + horizontal_resolution;
        for (const auto vertical_angle : vertical_angles) {
          geometry_msgs::msg::Vector3 rpy;
          rpy.x = 0;
          rpy.y = vertical_angle;
          rpy.z = horizontal_angle;
          auto quat = quaternion_operation::convertEulerAngleToQuaternion(rpy);
          directions.emplace_back(quat);
        }
      }
      directions_ = directions;
      previous_horizontal_angle_end_ = horizontal_angle_end;
      previous_horizontal_angle_start_ = horizontal_angle_start;
      previous_horizontal_resolution_ = horizontal_resolution;
      previous_vertical_angles_ = vertical_angles;
     std::cerr << "directions size: " << directions_.size() << std::endl;
    std::cerr << "start: " << previous_horizontal_angle_start_ << " " << horizontal_angle_start << std::endl;
    std::cerr << "end: " << previous_horizontal_angle_end_ << " " << horizontal_angle_end << std::endl;
    std::cerr << "res: " << previous_horizontal_resolution_ << " " << horizontal_resolution << std::endl;
    std::cerr << "ver size: " << previous_vertical_angles_.size() << " " << vertical_angles.size() << " matching: " << (previous_vertical_angles_ == vertical_angles) <<   std::endl;
    }
    return directions_;
  }

const sensor_msgs::msg::PointCloud2 Raycaster::raycast(
  std::string frame_id, const rclcpp::Time & stamp, geometry_msgs::msg::Pose origin,
  double horizontal_resolution, std::vector<double> vertical_angles, double horizontal_angle_start,
  double horizontal_angle_end, double max_distance, double min_distance)
{
  auto directions = getDirections(vertical_angles, horizontal_angle_start, horizontal_angle_end, horizontal_resolution);
  return raycast(frame_id, stamp, origin, directions, max_distance, min_distance);
}

const std::vector<std::string> & Raycaster::getDetectedObject() const { return detected_objects_; }

const sensor_msgs::msg::PointCloud2 Raycaster::raycast(
  std::string frame_id, const rclcpp::Time & stamp, geometry_msgs::msg::Pose origin,
  std::vector<geometry_msgs::msg::Quaternion> directions, double max_distance, double min_distance)
{
  detected_objects_ = {};
  std::vector<unsigned int> detected_ids = {};
  // scene_ = rtcNewScene(device_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto & pair : primitive_ptrs_) {
    auto id = pair.second->addToScene(device_, scene_);
    geometry_ids_.insert({id, pair.first});
  }
  rtcCommitScene(scene_);
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  for (const auto & direction : directions) {
    RTCRayHit rayhit = {};
    rayhit.ray.org_x = origin.position.x;
    rayhit.ray.org_y = origin.position.y;
    rayhit.ray.org_z = origin.position.z;
    // make raycast interact with all objects
    rayhit.ray.mask = 0b11111111'11111111'11111111'11111111;
    rayhit.ray.tfar = max_distance;
    rayhit.ray.tnear = min_distance;
    rayhit.ray.flags = false;
    const auto ray_direction = origin.orientation * direction;
    const auto rotation_mat = quaternion_operation::getRotationMatrix(ray_direction);
    const Eigen::Vector3d rotated_direction = rotation_mat * Eigen::Vector3d(1.0, 0.0, 0.0);
    rayhit.ray.dir_x = rotated_direction[0];
    rayhit.ray.dir_y = rotated_direction[1];
    rayhit.ray.dir_z = rotated_direction[2];
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rtcIntersect1(scene_, &context, &rayhit);
    if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
      double distance = rayhit.ray.tfar;
      const Eigen::Vector3d vector = quaternion_operation::getRotationMatrix(direction) *
                                     Eigen::Vector3d(1.0, 0.0, 0.0) * distance;
      pcl::PointXYZI p;
      {
        p.x = vector[0];
        p.y = vector[1];
        p.z = vector[2];
      }
      cloud->emplace_back(p);
      if (std::count(detected_ids.begin(), detected_ids.end(), rayhit.hit.geomID) == 0) {
        detected_ids.emplace_back(rayhit.hit.geomID);
      }
    }
  }
  for (const auto & id : detected_ids) {
    detected_objects_.emplace_back(geometry_ids_[id]);
  }

  for (const auto & id : geometry_ids_) {
    rtcDetachGeometry(scene_, id.first);
  }

  geometry_ids_.clear();
  primitive_ptrs_.clear();

  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*cloud, pointcloud_msg);
  // rtcReleaseScene(scene_);
  pointcloud_msg.header.frame_id = frame_id;
  pointcloud_msg.header.stamp = stamp;
  return pointcloud_msg;
}
}  // namespace simple_sensor_simulator
