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
  device_ = rtcNewDevice("frequency_level=simd128");
  scene_ = rtcNewScene(device_);
  rtcSetSceneFlags(scene_, RTC_SCENE_FLAG_DYNAMIC | RTC_SCENE_FLAG_ROBUST);
  rtcSetSceneBuildQuality(scene_, RTC_BUILD_QUALITY_LOW);
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

unsigned int Raycaster::addToScene(RTCDevice device, RTCScene scene, primitives::Primitive primitive)
{
  RTCGeometry mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  const auto transformed_vertices = primitive.getVertex();
  Vertex * vertices = static_cast<Vertex *>(rtcSetNewGeometryBuffer(
    mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex),
    transformed_vertices.size()));
  for (size_t i = 0; i < transformed_vertices.size(); i++) {
    vertices[i] = transformed_vertices[i];
  }
  const auto geometry_triangles = primitive.getTriangles();
  Triangle * triangles = static_cast<Triangle *>(rtcSetNewGeometryBuffer(
    mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), geometry_triangles.size()));
  for (size_t i = 0; i < geometry_triangles.size(); i++) {
    triangles[i] = geometry_triangles[i];
  }
  // enable raycasting
  rtcSetGeometryMask(mesh, 0b11111111'11111111'11111111'11111111);
  rtcCommitGeometry(mesh);
  unsigned int geometry_id = rtcAttachGeometry(scene, mesh);
  rtcReleaseGeometry(mesh);
  return geometry_id;
}

void Raycaster::updateOnScene(RTCScene scene, std::string entity_name, primitives::Primitive primitive)
{
  if (primitive_on_scene_id_.count(entity_name) == 0)
  {
    //TODO add to the scene when object not on scene
    std::cerr << "Please add the object " << entity_name <<" to the scene before calling updateOnScene function." << std::endl;
    return;
  }

  RTCGeometry geometry = rtcGetGeometry(scene, primitive_on_scene_id_[entity_name]);
  Vertex* geometry_vertices = (Vertex*) rtcGetGeometryBufferData(geometry, RTC_BUFFER_TYPE_VERTEX,0);
  std::vector<Vertex> primitive_vertices = primitive.getVertex();
  //TODO how to assert that primitive_vertices size == geometry_vertices size -> throw catch with removing object and adding it once more?
  for (size_t i=0; i < primitive_vertices.size(); ++i)
  {
    Vertex* current_vertex = &geometry_vertices[i];
    current_vertex->x = primitive_vertices[i].x;
    current_vertex->y = primitive_vertices[i].y;
    current_vertex->z = primitive_vertices[i].z;
  }

  rtcUpdateGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX, 0);
  rtcCommitGeometry(geometry);
}

std::vector<geometry_msgs::msg::Quaternion> Raycaster::getDirections(
    std::vector<double> vertical_angles, double horizontal_angle_start,
    double horizontal_angle_end, double horizontal_resolution)
  {
    if (directions_.empty() || previous_horizontal_angle_start_ != horizontal_angle_start ||
        previous_horizontal_angle_end_ != horizontal_angle_end || previous_horizontal_resolution_ != horizontal_resolution ||
        previous_vertical_angles_ != vertical_angles)
    {
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
  // scene_ = rtcNewScene(device_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto & pair : primitive_ptrs_) {
    if (primitive_on_scene_id_.count(pair.first) == 0)
    {
      auto id = addToScene(device_, scene_, *pair.second);
      primitive_on_scene_id_.insert_or_assign(pair.first, id);
    }
    else
    {
      updateOnScene(scene_, pair.first, *pair.second);
    }
  }

  // Run as many threads as physical cores (which is usually /2 virtual threads)
  // In heavy loads virtual threads (hyperthreading) add little to the overall performance
  // This also minimizes cost of creating a thread (roughly 10us on Intel/Linux)
  int thread_count = std::thread::hardware_concurrency() / 2;
  // Per thread data structures:
  std::vector<std::thread> threads(thread_count);
  std::vector<std::set<unsigned int>> thread_detected_ids(thread_count);
  std::vector<pcl::PointCloud<pcl::PointXYZI>> thread_cloud(thread_count);

  rtcCommitScene(scene_);
  RTCIntersectContext context;
  //TODO try not setting this
  for (int i = 0; i < threads.size(); ++i)
  {
    thread_cloud[i] = new pcl::PointCloud<pcl::PointXYZI>();
    threads[i] = std::thread(intersect, i, thread_count, scene_, thread_cloud[i], context, origin, thread_detected_ids[i], directions, max_distance, min_distance);
  }
    for (int i = 0; i < threads.size(); ++i)
  {
    threads[i].join();
    (*cloud) += thread_cloud[i];
  }
  for (auto&& detected_ids_in_thread : thread_detected_ids) {
      for (const auto & id : detected_ids_in_thread) {
          detected_objects_.emplace_back(geometry_ids_[id]);
      }
  }

  // for (const auto & id : geometry_ids_) {
  //   rtcDetachGeometry(scene_, id.first);
  // }

  geometry_ids_.clear();
  // primitive_ptrs_.clear();

  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*cloud, pointcloud_msg);
  // rtcReleaseScene(scene_);
  pointcloud_msg.header.frame_id = frame_id;
  pointcloud_msg.header.stamp = stamp;
  return pointcloud_msg;
}
}  // namespace simple_sensor_simulator
