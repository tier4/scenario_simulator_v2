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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_POSE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_POSE_HPP_

#include <lanelet2_matching/LaneletMatching.h>

#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>

namespace traffic_simulator
{
namespace lanelet_core
{
namespace pose
{
auto toMapPose(const traffic_simulator_msgs::msg::LaneletPose &, const bool fill_pitch = true)
  -> geometry_msgs::msg::PoseStamped;

auto toLaneletPose(
  const geometry_msgs::msg::Pose & pose, const bool include_crosswalk,
  const double matching_distance = 1.0) -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

auto toLaneletPose(
  const geometry_msgs::msg::Pose & pose, const lanelet::Ids &, const double matching_distance = 1.0)
  -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

auto toLaneletPose(
  const geometry_msgs::msg::Pose & pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance = 1.0) -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

auto toLaneletPose(
  const geometry_msgs::msg::Pose & pose, const lanelet::Id lanelet_id,
  const double matching_distance = 1.0) -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

auto toLaneletPoses(
  const geometry_msgs::msg::Pose & pose, const lanelet::Id lanelet_id,
  const double matching_distance = 5.0, const bool include_opposite_direction = true)
  -> std::vector<traffic_simulator_msgs::msg::LaneletPose>;

auto canonicalizeLaneletPose(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
  -> std::tuple<
    std::optional<traffic_simulator_msgs::msg::LaneletPose>, std::optional<lanelet::Id>>;

auto canonicalizeLaneletPose(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose,
  const lanelet::Ids & route_lanelets)
  -> std::tuple<
    std::optional<traffic_simulator_msgs::msg::LaneletPose>, std::optional<lanelet::Id>>;

auto getAllCanonicalizedLaneletPoses(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
  -> std::vector<traffic_simulator_msgs::msg::LaneletPose>;

// private for pose namespace
namespace
{
auto matchToLane(
  const geometry_msgs::msg::Pose & pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance = 1.0, const double reduction_ratio = 0.8)
  -> std::optional<lanelet::Id>;

auto excludeSubtypeLanelets(
  const std::vector<std::pair<double, lanelet::Lanelet>> & pair, const char subtype[])
  -> std::vector<std::pair<double, lanelet::Lanelet>>;

auto toPoint2d(const geometry_msgs::msg::Point & point) -> lanelet::BasicPoint2d;

auto absoluteHull(
  const lanelet::BasicPolygon2d & relative_hull, const lanelet::matching::Pose2d & pose)
  -> lanelet::BasicPolygon2d;

auto getNearbyLaneletIds(
  const geometry_msgs::msg::Point &, const double distance_threshold, const bool include_crosswalk,
  const std::size_t search_count = 5) -> lanelet::Ids;

auto getNearbyLaneletIds(
  const geometry_msgs::msg::Point &, const double distance_threshold,
  const std::size_t search_count = 5) -> lanelet::Ids;

auto getLeftLaneletIds(
  const lanelet::Id, const traffic_simulator_msgs::msg::EntityType &,
  const bool include_opposite_direction = true) -> lanelet::Ids;

auto getRightLaneletIds(
  const lanelet::Id lanelet_id, const traffic_simulator_msgs::msg::EntityType & entity_type,
  const bool include_opposite_direction = true) -> lanelet::Ids;
}  // namespace
}  // namespace pose
}  // namespace lanelet_core
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_POSE_HPP_
