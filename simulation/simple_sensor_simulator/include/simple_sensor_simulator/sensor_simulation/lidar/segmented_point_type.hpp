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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__SEGMENTED_POINT_TYPE_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__SEGMENTED_POINT_TYPE_HPP_

#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>
#include <limits>

/*
   Copy of `autoware::point_types` (autoware_core, common/autoware_point_types) at revision
   8c3fdfdffa57f5d7b62ca81ff512bec371bc98e8, so that this repository builds without autoware_core.
   Diff against that revision when re-syncing.
*/
namespace simple_sensor_simulator
{
enum class PointCloudClassification : std::uint8_t {
  CAR = 0,
  TRUCK = 1,
  BUS = 2,
  MOTORCYCLE = 3,
  BICYCLE = 4,
  PEDESTRIAN = 5,
  ANIMAL = 6,
  HAZARD = 7,
  FLAT_SURFACE = 8,
  STRUCTURE = 9,
  VEGETATION = 10,
  NOISE = 11,
  INVALID = 255,
};

struct PointXYZCPE
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  std::uint8_t class_id{static_cast<std::uint8_t>(PointCloudClassification::INVALID)};
  float probability{0.0f};
  float entropy{std::numeric_limits<float>::quiet_NaN()};
};

static_assert(
  sizeof(PointXYZCPE) == 24,
  "PointXYZCPE must stay a densely packed 24-byte point, because downstream consumers reject "
  "clouds whose point_step differs from sizeof(autoware::point_types::PointXYZCPE).");
static_assert(
  offsetof(PointXYZCPE, x) == 0 and offsetof(PointXYZCPE, y) == 4 and
    offsetof(PointXYZCPE, z) == 8 and offsetof(PointXYZCPE, class_id) == 12 and
    offsetof(PointXYZCPE, probability) == 16 and offsetof(PointXYZCPE, entropy) == 20,
  "PointXYZCPE must keep the field offsets of autoware::point_types::PointXYZCPE.");
}  // namespace simple_sensor_simulator

POINT_CLOUD_REGISTER_POINT_STRUCT(
  simple_sensor_simulator::PointXYZCPE,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, class_id, class_id)(
    float, probability, probability)(float, entropy, entropy))

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__SEGMENTED_POINT_TYPE_HPP_
