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
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>

namespace traffic_simulator
{
inline namespace pose
{
auto quietNaNPose() -> geometry_msgs::msg::Pose;

auto quietNaNLaneletPose() -> LaneletPose;

// Conversions
auto canonicalize(const LaneletPose & lanelet_pose) -> LaneletPose;

auto canonicalize(const LaneletPose & lanelet_pose, const lanelet::Ids & route_lanelets)
  -> LaneletPose;

auto toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> geometry_msgs::msg::Pose;

auto toMapPose(const LaneletPose & lanelet_pose) -> geometry_msgs::msg::Pose;

auto alternativeLaneletPoses(const LaneletPose & lanelet_pose) -> std::vector<LaneletPose>;

auto toCanonicalizedLaneletPose(const LaneletPose & lanelet_pose)
  -> std::optional<CanonicalizedLaneletPose>;

auto toCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose, const bool include_crosswalk)
  -> std::optional<CanonicalizedLaneletPose>;

auto toCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>;

auto toCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box,
  const lanelet::Ids & unique_route_lanelets, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>;

auto transformRelativePoseToGlobal(
  const geometry_msgs::msg::Pose & global_pose, const geometry_msgs::msg::Pose & relative_pose)
  -> geometry_msgs::msg::Pose;

// Relative msg::Pose
auto relativePose(const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>;

auto relativePose(const geometry_msgs::msg::Pose & from, const CanonicalizedLaneletPose & to)
  -> std::optional<geometry_msgs::msg::Pose>;

auto relativePose(const CanonicalizedLaneletPose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>;

auto boundingBoxRelativePose(
  const geometry_msgs::msg::Pose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const geometry_msgs::msg::Pose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box)
  -> std::optional<geometry_msgs::msg::Pose>;

// Relative LaneletPose
auto isAltitudeMatching(
  const CanonicalizedLaneletPose & lanelet_pose,
  const CanonicalizedLaneletPose & target_lanelet_pose) -> bool;

auto relativeLaneletPose(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  const RoutingConfiguration & routing_configuration) -> LaneletPose;

auto boundingBoxRelativeLaneletPose(
  const CanonicalizedLaneletPose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box,
  const RoutingConfiguration & routing_configuration) -> LaneletPose;

// Others
auto isInLanelet(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose, const lanelet::Id lanelet_id,
  const double tolerance) -> bool;

auto isInLanelet(const geometry_msgs::msg::Point & point, const lanelet::Id lanelet_id) -> bool;

auto isAtEndOfLanelets(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> bool;

auto findRoutableAlternativeLaneletPoseFrom(
  const lanelet::Id from_lanelet_id, const CanonicalizedLaneletPose & canonicalized_lanelet_pose,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box)
  -> std::optional<traffic_simulator::CanonicalizedLaneletPose>;

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
