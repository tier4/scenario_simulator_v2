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

#include <embree3/rtcore_device.h>
#include <embree3/rtcore_geometry.h>
#include <embree3/rtcore_ray.h>
#include <pmmintrin.h>
#include <quaternion_operation/quaternion_operation.h>
#include <xmmintrin.h>

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

struct Packet1Traits
{
  static inline constexpr std::size_t RayPacketSize = 1;
  using RTCRayHitType = RTCRayHit;
  static auto rtcIntersects(
    const int * /*valid*/, RTCScene scene, struct RTCIntersectContext * context,
    struct RTCRayHit * rayhit)
  {
    return rtcIntersect1(scene, context, rayhit);
  }
};

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
struct IntersectsCalculator
{
  static constexpr std::size_t RayPacketSize = RaycastTraits::RayPacketSize;
  using RTCRayHitType = typename RaycastTraits::RTCRayHitType;
  using RTCRayType = decltype(RTCRayHitType{}.ray);
  using RTCHitType = decltype(RTCRayHitType{}.hit);

  void initRayHits(
    RTCRayHitType & rayhits, [[maybe_unused]] std::size_t j,
    const geometry_msgs::msg::Pose & origin, const Eigen::Vector3d & rotated_direction,
    double max_distance, double min_distance)
  {
    if constexpr (RayPacketSize == 1) {
      rayhits.ray.org_x = origin.position.x;
      rayhits.ray.org_y = origin.position.y;
      rayhits.ray.org_z = origin.position.z;
      rayhits.ray.tnear = static_cast<float>(min_distance);

      rayhits.ray.dir_x = static_cast<float>(rotated_direction[0]);
      rayhits.ray.dir_y = static_cast<float>(rotated_direction[1]);
      rayhits.ray.dir_z = static_cast<float>(rotated_direction[2]);
      rayhits.ray.time = 0;

      rayhits.ray.tfar = static_cast<float>(max_distance);
      rayhits.ray.mask = -1;
      rayhits.ray.flags = false;

      rayhits.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    } else {
      rayhits.ray.org_x[j] = origin.position.x;
      rayhits.ray.org_y[j] = origin.position.y;
      rayhits.ray.org_z[j] = origin.position.z;
      rayhits.ray.tnear[j] = static_cast<float>(min_distance);

      rayhits.ray.dir_x[j] = static_cast<float>(rotated_direction[0]);
      rayhits.ray.dir_y[j] = static_cast<float>(rotated_direction[1]);
      rayhits.ray.dir_z[j] = static_cast<float>(rotated_direction[2]);
      rayhits.ray.time[j] = 0;

      rayhits.ray.tfar[j] = static_cast<float>(max_distance);
      rayhits.ray.mask[j] = -1;
      rayhits.ray.flags[j] = false;

      rayhits.hit.geomID[j] = RTC_INVALID_GEOMETRY_ID;
    }
  }

  auto getHitResult(const RTCRayHitType & rayhits, [[maybe_unused]] std::size_t j)
    -> std::pair<unsigned int, float>
  {
    if constexpr (RayPacketSize == 1) {
      return std::make_pair(rayhits.hit.geomID, rayhits.ray.tfar);
    } else {
      return std::make_pair(rayhits.hit.geomID[j], rayhits.ray.tfar[j]);
    }
  }

  auto operator()(
    RTCScene scene, RTCIntersectContext & context, const geometry_msgs::msg::Pose & origin,
    const std::vector<geometry_msgs::msg::Quaternion> & directions, double max_distance,
    double min_distance)
    -> std::tuple<std::unordered_set<unsigned int>, pcl::PointCloud<pcl::PointXYZI>::Ptr>
  {
    std::unordered_set<unsigned int> detected_ids = {};
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (auto i = 0u; i < directions.size(); i += RayPacketSize) {
      alignas(std::max(4 * RayPacketSize, alignof(int))) std::array<int, RayPacketSize> valid = {};
      alignas(std::max(4 * RayPacketSize, alignof(RTCRayHitType))) RTCRayHitType rayhits{};

      for (auto j = 0u; j < RayPacketSize && i + j < directions.size(); ++j) {
        const auto & direction = directions[i + j];
        const auto ray_direction = origin.orientation * direction;
        const Eigen::Vector3d rotated_direction = getDirectionVector(ray_direction);
        initRayHits(rayhits, j, origin, rotated_direction, max_distance, min_distance);
        valid[j] = -1;
      }

      RaycastTraits::rtcIntersects(valid.data(), scene, &context, &rayhits);

      for (auto j = 0u; j < RayPacketSize && i + j < directions.size(); ++j) {
        auto [geomID, distance] = getHitResult(rayhits, j);
        if (geomID != RTC_INVALID_GEOMETRY_ID) {
          const auto & direction = directions[i + j];
          const Eigen::Vector3d vector = getDirectionVector(direction) * distance;
          pcl::PointXYZI p;
          {
            p.x = vector[0];
            p.y = vector[1];
            p.z = vector[2];
          }
          cloud->emplace_back(p);
          detected_ids.insert(geomID);
        }
      }
    }

    return std::make_tuple(std::move(detected_ids), std::move(cloud));
  }
};

auto calcIntersect1M(
  RTCScene scene, RTCIntersectContext & context, const geometry_msgs::msg::Pose & origin,
  const std::vector<geometry_msgs::msg::Quaternion> & directions, double max_distance,
  double min_distance)
  -> std::tuple<std::unordered_set<unsigned int>, pcl::PointCloud<pcl::PointXYZI>::Ptr>
{
  std::vector<RTCRayHit> rayhits(directions.size());
  for (auto i = 0u; i < directions.size(); ++i) {
    const auto ray_direction = origin.orientation * directions[i];
    const Eigen::Vector3d rotated_direction = getDirectionVector(ray_direction);

    auto & rayhit = rayhits[i];
    rayhit.ray.org_x = origin.position.x;
    rayhit.ray.org_y = origin.position.y;
    rayhit.ray.org_z = origin.position.z;
    rayhit.ray.tnear = static_cast<float>(min_distance);

    rayhit.ray.dir_x = static_cast<float>(rotated_direction[0]);
    rayhit.ray.dir_y = static_cast<float>(rotated_direction[1]);
    rayhit.ray.dir_z = static_cast<float>(rotated_direction[2]);
    rayhit.ray.time = 0;

    rayhit.ray.tfar = static_cast<float>(max_distance);
    rayhit.ray.mask = -1;
    rayhit.ray.flags = false;
  }

  rtcIntersect1M(scene, &context, rayhits.data(), rayhits.size(), sizeof(RTCRayHit));

  std::unordered_set<unsigned int> detected_ids = {};
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

  for (auto i = 0u; i < directions.size(); ++i) {
    auto geomID = rayhits[i].hit.geomID;
    auto distance = rayhits[i].ray.tfar;
    if (geomID != RTC_INVALID_GEOMETRY_ID) {
      const auto & direction = directions[i];
      const Eigen::Vector3d vector = getDirectionVector(direction) * distance;
      pcl::PointXYZI p;
      {
        p.x = vector[0];
        p.y = vector[1];
        p.z = vector[2];
      }
      cloud->emplace_back(p);
      detected_ids.insert(geomID);
    }
  }

  return std::make_tuple(std::move(detected_ids), std::move(cloud));
}

Raycaster::Raycaster() : Raycaster("threads=1,set_affinity=1") {}

Raycaster::Raycaster(const std::string & embree_config)
: primitive_ptrs_{},
  device_(rtcNewDevice(embree_config.c_str())),
  scene_(nullptr),
  engine_(seed_gen_())
{
  if (rtcGetDeviceProperty(device_, RTC_DEVICE_PROPERTY_NATIVE_RAY16_SUPPORTED)) {
    calc_intersects = IntersectsCalculator<Packet16Traits>{};
  } else if (rtcGetDeviceProperty(device_, RTC_DEVICE_PROPERTY_NATIVE_RAY8_SUPPORTED)) {
    calc_intersects = IntersectsCalculator<Packet8Traits>{};
  } else if (rtcGetDeviceProperty(device_, RTC_DEVICE_PROPERTY_NATIVE_RAY4_SUPPORTED)) {
    calc_intersects = IntersectsCalculator<Packet4Traits>{};
  } else {
    calc_intersects = IntersectsCalculator<Packet1Traits>{};
  }

  _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
}

Raycaster::~Raycaster() { rtcReleaseDevice(device_); }

sensor_msgs::msg::PointCloud2 Raycaster::raycast(
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

sensor_msgs::msg::PointCloud2 Raycaster::raycast(
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
  context.flags = RTC_INTERSECT_CONTEXT_FLAG_COHERENT;

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
