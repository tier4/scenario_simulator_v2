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

#include <geometry/bounding_box.hpp>
#include <geometry/distance.hpp>
#include <geometry/transform.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>

namespace traffic_simulator
{

namespace distance
{
using CanonicalizedLaneletPose = lanelet_pose::CanonicalizedLaneletPose;
using CanonicalizedEntityStatus = entity_status::CanonicalizedEntityStatus;

// Pose
/// @todo: they will be moved to separate class for "poses"
auto canonicalize(
  const LaneletPose & lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> CanonicalizedLaneletPose;

auto toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> const geometry_msgs::msg::Pose;

auto toLaneletPose(
  const geometry_msgs::msg::Pose & map_pose, bool include_crosswalk,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>;

// RelativePose
///@todo: they will be moved to separate class for "poses"
auto getRelativePose(const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>;

auto getRelativePose(const geometry_msgs::msg::Pose & from, const CanonicalizedLaneletPose & to)
  -> std::optional<geometry_msgs::msg::Pose>;

auto getRelativePose(const CanonicalizedLaneletPose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>;

///@todo: it will be moved to separate class for "poses"
auto getBoundingBoxRelativePose(
  const geometry_msgs::msg::Pose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const geometry_msgs::msg::Pose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox)
  -> std::optional<geometry_msgs::msg::Pose>;

///@todo: it will be moved to separate class for "poses"
auto makeNativeRelativeLanePosition(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose;

///@todo: it will be moved to separate class for "poses"
auto makeNativeBoundingBoxRelativeLanePosition(
  const CanonicalizedLaneletPose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const CanonicalizedLaneletPose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose;
}  // namespace distance

namespace distance
{

using CanonicalizedLaneletPose = lanelet_pose::CanonicalizedLaneletPose;
using CanonicalizedEntityStatus = entity_status::CanonicalizedEntityStatus;

// Lateral
auto getLateralDistance(
  const lanelet_pose::CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<double>;

auto getLateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  double matching_distance, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

// Longitudinal
auto getLongitudinalDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool include_adjacent_lanelet, bool include_opposite_direction, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

// BoundingBox
auto getBoundingBoxDistance(
  const geometry_msgs::msg::Pose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const geometry_msgs::msg::Pose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox)
  -> std::optional<double>;

auto getBoundingBoxLaneLateralDistance(
  const CanonicalizedLaneletPose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const CanonicalizedLaneletPose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<double>;

auto getBoundingBoxLaneLongitudinalDistance(
  const CanonicalizedLaneletPose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const CanonicalizedLaneletPose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox,
  bool include_adjacent_lanelet, bool include_opposite_direction, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

// Bounds
auto getDistanceToLaneBound(
  const CanonicalizedEntityStatus & status, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

auto getDistanceToLaneBound(
  const CanonicalizedEntityStatus & status, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

auto getDistanceToLeftLaneBound(
  const CanonicalizedEntityStatus & status, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

auto getDistanceToLeftLaneBound(
  const CanonicalizedEntityStatus & status, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

auto getDistanceToRightLaneBound(
  const CanonicalizedEntityStatus & status, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

auto getDistanceToRightLaneBound(
  const CanonicalizedEntityStatus & status, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double;

// Other objects
auto getDistanceToCrosswalk(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints_array,
  const lanelet::Id target_crosswalk_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

auto getDistanceToStopLine(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints_array,
  const lanelet::Id target_stop_line_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>;

}  // namespace distance
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__DISTANCE_HPP_
