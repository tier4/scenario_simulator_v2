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

#include <embree3/rtcore_device.h>
#include <embree3/rtcore_geometry.h>
#include <embree3/rtcore_ray.h>
#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <simple_sensor_simulator/sensor_simulation/lidar/lidar_sensor.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace simple_sensor_simulator
{
auto getDirectionVector(const geometry_msgs::msg::Quaternion & quat)
{
  double x = quat.x;
  double y = quat.y;
  double z = quat.z;
  double w = quat.w;
  return Eigen::Vector3d(x * x - y * y - z * z + w * w, 2 * (x * y + z * w), 2 * (z * x - w * y));
}

struct Packet4Traits
{
  static inline constexpr std::size_t RayPacketSize = 4;
  using RTCRayHitType = RTCRayHit4;
  static inline constexpr auto rtcIntersects = rtcIntersect4;
};

struct Packet8Traits
{
  static inline constexpr std::size_t RayPacketSize = 8;
  using RTCRayHitType = RTCRayHit8;
  static inline constexpr auto rtcIntersects = rtcIntersect8;
};

struct Packet16Traits
{
  static inline constexpr std::size_t RayPacketSize = 16;
  using RTCRayHitType = RTCRayHit16;
  static inline constexpr auto rtcIntersects = rtcIntersect16;
};

template <typename RaycastTraits>
auto calcIntersects(
  RTCScene scene, RTCIntersectContext & context, const geometry_msgs::msg::Pose & origin,
  const std::vector<geometry_msgs::msg::Quaternion> & directions, double max_distance,
  double min_distance)
  -> std::tuple<std::vector<unsigned int>, pcl::PointCloud<pcl::PointXYZI>::Ptr>
{
  constexpr std::size_t RayPacketSize = RaycastTraits::RayPacketSize;
  using RTCRayHitType = typename RaycastTraits::RTCRayHitType;
  constexpr auto rtcIntersects = RaycastTraits::rtcIntersects;

  std::vector<unsigned int> detected_ids = {};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

  for (auto i = 0u; i < directions.size(); i += RayPacketSize) {
    alignas(4 * RayPacketSize) std::array<int, RayPacketSize> valid = {};
    alignas(4 * RayPacketSize) RTCRayHitType rayhits;

    for (auto j = 0u; j < RayPacketSize && i + j < directions.size(); ++j) {
      const auto & direction = directions[i + j];
      rayhits.ray.org_x[j] = origin.position.x;
      rayhits.ray.org_y[j] = origin.position.y;
      rayhits.ray.org_z[j] = origin.position.z;
      rayhits.ray.tfar[j] = static_cast<float>(max_distance);
      rayhits.ray.tnear[j] = static_cast<float>(min_distance);
      rayhits.ray.flags[j] = false;
      const auto ray_direction = origin.orientation * direction;
      const Eigen::Vector3d rotated_direction = getDirectionVector(ray_direction);
      rayhits.ray.dir_x[j] = static_cast<float>(rotated_direction[0]);
      rayhits.ray.dir_y[j] = static_cast<float>(rotated_direction[1]);
      rayhits.ray.dir_z[j] = static_cast<float>(rotated_direction[2]);
      rayhits.hit.geomID[j] = RTC_INVALID_GEOMETRY_ID;

      valid[j] = -1;
    }

    rtcIntersects(valid.data(), scene, &context, &rayhits);

    for (auto j = 0u; j < RayPacketSize && i + j < directions.size(); ++j) {
      if (rayhits.hit.geomID[j] != RTC_INVALID_GEOMETRY_ID) {
        const auto & direction = directions[i + j];
        double distance = rayhits.ray.tfar[j];
        const Eigen::Vector3d vector = getDirectionVector(direction) * distance;
        pcl::PointXYZI p;
        {
          p.x = vector[0];
          p.y = vector[1];
          p.z = vector[2];
        }
        cloud->emplace_back(p);
        if (std::count(detected_ids.begin(), detected_ids.end(), rayhits.hit.geomID[j]) == 0) {
          detected_ids.emplace_back(rayhits.hit.geomID[j]);
        }
      }
    }
  }

  return std::make_tuple(std::move(detected_ids), std::move(cloud));
}

Raycaster::Raycaster() : primitive_ptrs_(0), device_(nullptr), scene_(nullptr), engine_(seed_gen_())
{
  device_ = rtcNewDevice(nullptr);

  if (rtcGetDeviceProperty(device_, RTC_DEVICE_PROPERTY_NATIVE_RAY16_SUPPORTED)) {
    calc_intersects = calcIntersects<Packet16Traits>;
  } else if (rtcGetDeviceProperty(device_, RTC_DEVICE_PROPERTY_NATIVE_RAY8_SUPPORTED)) {
    calc_intersects = calcIntersects<Packet8Traits>;
  } else /*if (rtcGetDeviceProperty(device_, RTC_DEVICE_PROPERTY_NATIVE_RAY4_SUPPORTED))*/ {
    calc_intersects = calcIntersects<Packet4Traits>;
  }
}

Raycaster::Raycaster(std::string embree_config)
: primitive_ptrs_(0), device_(nullptr), scene_(nullptr), engine_(seed_gen_())
{
  device_ = rtcNewDevice(embree_config.c_str());
}

Raycaster::~Raycaster() { rtcReleaseDevice(device_); }

const sensor_msgs::msg::PointCloud2 Raycaster::raycast(
  std::string frame_id, const rclcpp::Time & stamp, geometry_msgs::msg::Pose origin,
  double horizontal_resolution, const std::vector<double> & vertical_angles,
  double horizontal_angle_start, double horizontal_angle_end, double max_distance,
  double min_distance)
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
  return raycast(std::move(frame_id), stamp, origin, directions, max_distance, min_distance);
}

const std::vector<std::string> & Raycaster::getDetectedObject() const { return detected_objects_; }

const sensor_msgs::msg::PointCloud2 Raycaster::raycast(
  std::string frame_id, const rclcpp::Time & stamp, geometry_msgs::msg::Pose origin,
  const std::vector<geometry_msgs::msg::Quaternion> & directions, double max_distance,
  double min_distance)
{
  detected_objects_ = {};
  scene_ = rtcNewScene(device_);
  for (auto & pair : primitive_ptrs_) {
    auto id = pair.second->addToScene(device_, scene_);
    geometry_ids_.insert({id, pair.first});
  }
  rtcCommitScene(scene_);

  RTCIntersectContext context;
  rtcInitIntersectContext(&context);

  auto [detected_ids, cloud] =
    calc_intersects(scene_, context, origin, directions, max_distance, min_distance);

  for (const auto & id : detected_ids) {
    detected_objects_.emplace_back(geometry_ids_[id]);
  }
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*cloud, pointcloud_msg);
  rtcReleaseScene(scene_);
  pointcloud_msg.header.frame_id = std::move(frame_id);
  pointcloud_msg.header.stamp = stamp;
  return pointcloud_msg;
}
}  // namespace simple_sensor_simulator
