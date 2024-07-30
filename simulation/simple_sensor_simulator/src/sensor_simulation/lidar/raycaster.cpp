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
#include <iostream>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace simple_sensor_simulator
{
Raycaster::Raycaster()
: primitive_ptrs_(0),
  device_(rtcNewDevice(nullptr)),
  scene_(rtcNewScene(device_)),
  engine_(seed_gen_())
{
}

Raycaster::Raycaster(std::string embree_config)
: primitive_ptrs_(0),
  device_(rtcNewDevice(embree_config.c_str())),
  scene_(rtcNewScene(device_)),
  engine_(seed_gen_())
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

const std::vector<std::string> & Raycaster::getDetectedObject() const { return detected_objects_; }

const sensor_msgs::msg::PointCloud2 Raycaster::raycast(
  const std::string & frame_id, const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & origin,
  double max_distance, double min_distance)
{
  detected_objects_ = {};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto & pair : primitive_ptrs_) {
    auto id = pair.second->addToScene(device_, scene_);
    geometry_ids_.insert({id, pair.first});
  }

  // Run as many threads as physical cores (which is usually /2 virtual threads)
  // In heavy loads virtual threads (hyper-threading) add little to the overall performance
  // This also minimizes cost of creating a thread (roughly 10us on Intel/Linux)
  int thread_count = std::thread::hardware_concurrency() / 2;
  // Per thread data structures:
  std::vector<std::thread> threads(thread_count);
  std::vector<std::set<unsigned int>> thread_detected_ids(thread_count);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> thread_cloud(thread_count);

  rtcCommitScene(scene_);
  for (unsigned int i = 0; i < threads.size(); ++i) {
    thread_cloud[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    threads[i] = std::thread(
      intersect, i, thread_count, scene_, thread_cloud[i], origin, std::ref(thread_detected_ids[i]),
      max_distance, min_distance, std::ref(rotation_matrices_));
  }
  for (unsigned int i = 0; i < threads.size(); ++i) {
    threads[i].join();
    (*cloud) += *(thread_cloud[i]);
  }
  for (auto && detected_ids_in_thread : thread_detected_ids) {
    for (const auto & id : detected_ids_in_thread) {
      detected_objects_.emplace_back(geometry_ids_[id]);
    }
  }

  for (const auto & id : geometry_ids_) {
    rtcDetachGeometry(scene_, id.first);
  }

  geometry_ids_.clear();
  primitive_ptrs_.clear();

  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*cloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = frame_id;
  pointcloud_msg.header.stamp = stamp;
  return pointcloud_msg;
}
}  // namespace simple_sensor_simulator
