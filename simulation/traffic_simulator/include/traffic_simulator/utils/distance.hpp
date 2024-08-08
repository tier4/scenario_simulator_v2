// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__UTILS__DISTANCE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__DISTANCE_HPP_

#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>

namespace traffic_simulator
{
inline namespace distance
{
// Lateral
auto lateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<double>;

auto lateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  double matching_distance, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

// Lateral (unit: lanes)
auto countLaneChanges(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<std::pair<int, int>>;

// Longitudinal
auto longitudinalDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool include_adjacent_lanelet, bool include_opposite_direction, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

// BoundingBox
auto boundingBoxDistance(
  const geometry_msgs::msg::Pose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const geometry_msgs::msg::Pose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box) -> std::optional<double>;

auto boundingBoxLaneLateralDistance(
  const CanonicalizedLaneletPose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

auto boundingBoxLaneLongitudinalDistance(
  const CanonicalizedLaneletPose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box, bool include_adjacent_lanelet,
  bool include_opposite_direction, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

// Bounds
auto distanceToLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

auto distanceToLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

auto distanceToLeftLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

auto distanceToLeftLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

auto distanceToRightLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

auto distanceToRightLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

// Other objects
auto distanceToCrosswalk(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints_array,
  const lanelet::Id target_crosswalk_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

auto distanceToStopLine(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints_array,
  const lanelet::Id target_stop_line_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;
}  // namespace distance
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__DISTANCE_HPP_
