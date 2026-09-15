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
#include <cstddef>
#include <get_parameter/get_parameter.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/object_compatibility.hpp>
#include <simple_sensor_simulator/sensor_simulation/lidar/segmented_point_cloud.hpp>

namespace simple_sensor_simulator
{
auto classificationOf(const traffic_simulator_msgs::EntityStatus & entity_status)
  -> PointCloudClassification
{
  // No TRAILER here, and upstream try_into_pointcloud maps that label onto TRUCK too.
  switch (entity_status.subtype().value()) {
    case traffic_simulator_msgs::EntitySubtype::CAR:
      return PointCloudClassification::CAR;
    case traffic_simulator_msgs::EntitySubtype::TRUCK:
    case traffic_simulator_msgs::EntitySubtype::TRAILER:
      return PointCloudClassification::TRUCK;
    case traffic_simulator_msgs::EntitySubtype::BUS:
      return PointCloudClassification::BUS;
    case traffic_simulator_msgs::EntitySubtype::MOTORCYCLE:
      return PointCloudClassification::MOTORCYCLE;
    case traffic_simulator_msgs::EntitySubtype::BICYCLE:
      return PointCloudClassification::BICYCLE;
    case traffic_simulator_msgs::EntitySubtype::PEDESTRIAN:
      return PointCloudClassification::PEDESTRIAN;
    default:
      break;
  }

  /*
     The subtype is unset or unknown, so fall back to the type.

     MISC_OBJECT is STRUCTURE, not HAZARD: HAZARD is object compatible, so those points would never
     reach this channel. Real perception labels a barrier STRUCTURE for the same reason.
  */
  switch (entity_status.type().type()) {
    case traffic_simulator_msgs::EntityType::EGO:
    case traffic_simulator_msgs::EntityType::VEHICLE:
      return PointCloudClassification::CAR;
    case traffic_simulator_msgs::EntityType::PEDESTRIAN:
      return PointCloudClassification::PEDESTRIAN;
    case traffic_simulator_msgs::EntityType::MISC_OBJECT:
      return PointCloudClassification::STRUCTURE;
    default:
      return PointCloudClassification::INVALID;
  }
}

auto toSegmentedPointCloud(const Raycaster::RaycastResult & result) -> pcl::PointCloud<PointXYZCPE>
{
  pcl::PointCloud<PointXYZCPE> segmented_cloud;

  const auto & cloud = result.cloud;
  if (not cloud) {
    return segmented_cloud;
  }

  const auto & point_to_entity_index = result.point_to_entity_index;
  const auto & raycast_entities = result.raycast_entities;

  if (point_to_entity_index.size() != cloud->size()) {
    RCLCPP_WARN_STREAM_ONCE(
      common::getParameterNode().get_logger(), "Only " << point_to_entity_index.size() << " of "
                                                       << cloud->size()
                                                       << " raycast points can be classified.");
  }

  // Rejected points are dropped, not reported with a placeholder class a consumer cannot spot.
  segmented_cloud.points.reserve(cloud->size());
  for (std::size_t i = 0; i < std::min(cloud->size(), point_to_entity_index.size()); ++i) {
    const auto entity_index = point_to_entity_index[i];
    if (entity_index >= raycast_entities.size()) {
      continue;
    }
    const auto classification = classificationOf(raycast_entities[entity_index].entity_status);
    if (
      classification == PointCloudClassification::INVALID or is_object_compatible(classification)) {
      continue;
    }

    PointXYZCPE point;
    point.x = cloud->points[i].x;
    point.y = cloud->points[i].y;
    point.z = cloud->points[i].z;
    point.class_id = static_cast<std::uint8_t>(classification);
    // A simulated classification is exact, so the confidence is maximal.
    point.probability = 1.0f;
    point.entropy = 0.0f;

    segmented_cloud.points.push_back(point);
  }

  segmented_cloud.width = segmented_cloud.points.size();
  segmented_cloud.height = 1;
  segmented_cloud.is_dense = cloud->is_dense;
  return segmented_cloud;
}
}  // namespace simple_sensor_simulator
