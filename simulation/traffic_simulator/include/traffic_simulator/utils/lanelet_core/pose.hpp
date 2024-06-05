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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_POSE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_POSE_HPP_

#include <lanelet2_matching/LaneletMatching.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>

namespace traffic_simulator
{
namespace lanelet_core
{
namespace pose
{
using Point = geometry_msgs::msg::Point;
using Vector3 = geometry_msgs::msg::Vector3;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using BoundingBox = traffic_simulator_msgs::msg::BoundingBox;
using LaneletPose = traffic_simulator_msgs::msg::LaneletPose;
using EntityType = traffic_simulator_msgs::msg::EntityType;

auto toMapPose(const LaneletPose &, const bool fill_pitch = true) -> PoseStamped;

auto toLaneletPose(const Pose & pose, const bool include_crosswalk, const double matching_distance)
  -> std::optional<LaneletPose>;

auto toLaneletPose(const Pose & pose, const lanelet::Ids &, const double matching_distance)
  -> std::optional<LaneletPose>;

auto toLaneletPose(
  const Pose & pose, const BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance) -> std::optional<LaneletPose>;

auto toLaneletPose(const Pose & pose, const lanelet::Id lanelet_id, const double matching_distance)
  -> std::optional<LaneletPose>;

auto toLaneletPoses(
  const Pose & pose, const lanelet::Id lanelet_id, const double matching_distance,
  const bool include_opposite_direction) -> std::vector<LaneletPose>;

auto canonicalizeLaneletPose(const LaneletPose & lanelet_pose)
  -> std::tuple<std::optional<LaneletPose>, std::optional<lanelet::Id>>;

auto canonicalizeLaneletPose(const LaneletPose & lanelet_pose, const lanelet::Ids & route_lanelets)
  -> std::tuple<std::optional<LaneletPose>, std::optional<lanelet::Id>>;

auto getAlternativeLaneletPoses(const LaneletPose & lanelet_pose) -> std::vector<LaneletPose>;

auto getAlongLaneletPose(const LaneletPose & from_pose, const double along) -> LaneletPose;

// private
namespace
{
auto matchToLane(
  const Pose & pose, const BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance, const double reduction_ratio) -> std::optional<lanelet::Id>;

auto getLeftLaneletIds(const lanelet::Id, const EntityType &, const bool include_opposite_direction)
  -> lanelet::Ids;

auto getRightLaneletIds(
  const lanelet::Id lanelet_id, const EntityType & entity_type,
  const bool include_opposite_direction) -> lanelet::Ids;
}  // namespace
}  // namespace pose
}  // namespace lanelet_core
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_POSE_HPP_
