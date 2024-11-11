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

#ifndef TRAFFIC_SIMULATOR__UTILS__POSE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__POSE_HPP_

#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>

namespace traffic_simulator
{
inline namespace pose
{
using Point = geometry_msgs::msg::Point;
using Vector3 = geometry_msgs::msg::Vector3;
using Pose = geometry_msgs::msg::Pose;
using BoundingBox = traffic_simulator_msgs::msg::BoundingBox;

auto quietNaNPose() -> Pose;

auto quietNaNLaneletPose() -> LaneletPose;

auto isInLanelet(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose, const lanelet::Id lanelet_id,
  const double tolerance) -> bool;

auto isInLanelet(const Point & point, const lanelet::Id lanelet_id) -> bool;

auto isAtEndOfLanelets(const CanonicalizedLaneletPose & canonicalized_lanelet_pose) -> bool;

// Conversions
auto toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> Pose;

auto toMapPose(const LaneletPose & lanelet_pose) -> Pose;

auto canonicalize(const LaneletPose & lanelet_pose) -> LaneletPose;

auto canonicalize(const LaneletPose & lanelet_pose, const lanelet::Ids & route_lanelets)
  -> LaneletPose;

auto alternativeLaneletPoses(const LaneletPose & lanelet_pose) -> std::vector<LaneletPose>;

auto toCanonicalizedLaneletPose(const LaneletPose & lanelet_pose)
  -> std::optional<CanonicalizedLaneletPose>;

auto toCanonicalizedLaneletPose(const Pose & map_pose, const bool include_crosswalk)
  -> std::optional<CanonicalizedLaneletPose>;

auto toCanonicalizedLaneletPose(
  const Point & map_point, const BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>;

auto toCanonicalizedLaneletPose(
  const Pose & map_pose, const BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>;

auto toCanonicalizedLaneletPose(
  const Pose & map_pose, const BoundingBox & bounding_box,
  const lanelet::Ids & unique_route_lanelets, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>;

auto transformRelativePoseToGlobal(const Pose & global_pose, const Pose & relative_pose) -> Pose;

// Relative msg::Pose
auto relativePose(const Pose & from, const Pose & to) -> std::optional<Pose>;

auto relativePose(const Pose & from, const CanonicalizedLaneletPose & to) -> std::optional<Pose>;

auto relativePose(const CanonicalizedLaneletPose & from, const Pose & to) -> std::optional<Pose>;

auto boundingBoxRelativePose(
  const Pose & from, const BoundingBox & from_bounding_box, const Pose & to,
  const BoundingBox & to_bounding_box) -> std::optional<Pose>;

// Relative LaneletPose
auto relativeLaneletPose(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  const bool allow_lane_change) -> LaneletPose;

auto boundingBoxRelativeLaneletPose(
  const CanonicalizedLaneletPose & from, const BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & to, const BoundingBox & to_bounding_box,
  const bool allow_lane_change) -> LaneletPose;

// Others
namespace pedestrian
{
auto transformToCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box,
  const lanelet::Ids & unique_route_lanelets, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>;
}  // namespace pedestrian
}  // namespace pose
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__POSE_HPP_
