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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__OBJECT_COMPATIBILITY_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__OBJECT_COMPATIBILITY_HPP_

#include <simple_sensor_simulator/sensor_simulation/lidar/segmented_point_type.hpp>

/*
   Copy of `autoware::object_recognition_utils::is_object_compatible` (autoware_core,
   common/autoware_object_recognition_utils) at revision
   8c3fdfdffa57f5d7b62ca81ff512bec371bc98e8. A file of its own so that each copied upstream package
   can be diffed against one header.
*/
namespace simple_sensor_simulator
{
/// @brief Whether a classification belongs to the detected objects rather than to the point cloud.
constexpr auto is_object_compatible(const PointCloudClassification classification) -> bool
{
  switch (classification) {
    case PointCloudClassification::CAR:
    case PointCloudClassification::TRUCK:
    case PointCloudClassification::BUS:
    case PointCloudClassification::MOTORCYCLE:
    case PointCloudClassification::BICYCLE:
    case PointCloudClassification::PEDESTRIAN:
    case PointCloudClassification::ANIMAL:
    case PointCloudClassification::HAZARD:
      return true;
    case PointCloudClassification::FLAT_SURFACE:
    case PointCloudClassification::STRUCTURE:
    case PointCloudClassification::VEGETATION:
    case PointCloudClassification::NOISE:
    case PointCloudClassification::INVALID:
      return false;
  }
  // A label added upstream since this copy. Upstream treats it as non-object too.
  return false;
}
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__OBJECT_COMPATIBILITY_HPP_
