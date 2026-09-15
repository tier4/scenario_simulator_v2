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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__SEGMENTED_POINT_CLOUD_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__SEGMENTED_POINT_CLOUD_HPP_

#include <simple_sensor_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/segmented_point_type.hpp>

namespace simple_sensor_simulator
{
/// @brief The classification implied by an entity's type and subtype.
auto classificationOf(const traffic_simulator_msgs::EntityStatus & entity_status)
  -> PointCloudClassification;

/// @brief Copy the raycast points into a `PointXYZCPE` cloud, dropping the object compatible ones.
auto toSegmentedPointCloud(const Raycaster::RaycastResult & result) -> pcl::PointCloud<PointXYZCPE>;
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__LIDAR__SEGMENTED_POINT_CLOUD_HPP_
