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

#ifndef TRAFFIC_SIMULATOR__LANELET_WRAPPER_POSE_HPP_
#define TRAFFIC_SIMULATOR__LANELET_WRAPPER_POSE_HPP_

#include <lanelet2_matching/LaneletMatching.h>

#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace pose
{
/// @note This value was determined experimentally by @hakuturu583 and not theoretically.
/// @sa https://github.com/tier4/scenario_simulator_v2/commit/4c8e9f496b061b00bec799159d59c33f2ba46b3a
constexpr static double DEFAULT_MATCH_TO_LANE_REDUCTION_RATIO = 0.8;

auto toMapPose(const LaneletPose & lanelet_pose, const bool fill_pitch = true) -> PoseStamped;

auto isAltitudeMatching(const double current_altitude, const double target_altitude) -> bool;

auto toLaneletPose(
  const Pose & map_pose, const lanelet::Id lanelet_id, const double matching_distance = 1.0)
  -> std::optional<LaneletPose>;

auto toLaneletPose(
  const Pose & map_pose, const lanelet::Ids & lanelet_ids, const double matching_distance = 1.0)
  -> std::optional<LaneletPose>;

auto toLaneletPose(
  const Pose & map_pose, const bool include_crosswalk, const double matching_distance = 1.0)
  -> std::optional<LaneletPose>;

auto toLaneletPose(
  const Pose & map_pose, const BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance = 1.0,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type)
  -> std::optional<LaneletPose>;

auto toLaneletPoses(
  const Pose & map_pose, const lanelet::Id lanelet_id, const double matching_distance = 5.0,
  const bool include_opposite_direction = true,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type)
  -> std::vector<LaneletPose>;

/**
 * @brief Retrieves alternative lanelet poses based on the reference lanelet pose.
 * 
 * This method computes alternative lanelet poses in the previous and next lanelets 
 * relative to a given reference lanelet pose. It recursively explores the neighboring 
 * lanelets until no further alternatives are found. The decision of whether a pose belongs
 * to a previous or next lanelet is based on the `s` value of the reference pose:
 * - If `s` is negative, the pose is assumed to be on the previous lanelet.
 * - If `s` exceeds the lanelet length, the pose is assumed to be on the next lanelet.
 * - If `s` is within the valid range of the lanelet (from 0 to the lanelet's length), 
 *   the reference lanelet pose is returned directly.
 * 
 * @param reference_lanelet_pose The reference pose on a lanelet, used to determine its position 
 *                                and compute alternatives in neighboring lanelets.
 * 
 * @return A vector of alternative `LaneletPose` objects representing poses in the neighboring
 *         lanelets, or the reference pose if no alternatives are found.
 */
auto alternativeLaneletPoses(const LaneletPose & reference_lanelet_pose)
  -> std::vector<LaneletPose>;

auto alongLaneletPose(
  const LaneletPose & from_pose, const lanelet::Ids & route_lanelets, const double distance)
  -> LaneletPose;

auto alongLaneletPose(
  const LaneletPose & from_pose, const double distance,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> LaneletPose;

auto canonicalizeLaneletPose(const LaneletPose & lanelet_pose)
  -> std::tuple<std::optional<LaneletPose>, std::optional<lanelet::Id>>;

auto canonicalizeLaneletPose(const LaneletPose & lanelet_pose, const lanelet::Ids & route_lanelets)
  -> std::tuple<std::optional<LaneletPose>, std::optional<lanelet::Id>>;

auto findMatchingLanes(
  const geometry_msgs::msg::Pose &, const traffic_simulator_msgs::msg::BoundingBox &,
  const bool include_crosswalk, const double matching_distance = 1.0,
  const double reduction_ratio = DEFAULT_MATCH_TO_LANE_REDUCTION_RATIO,
  const traffic_simulator::RoutingGraphType type =
    traffic_simulator::RoutingConfiguration().routing_graph_type)
  -> std::optional<std::set<std::pair<double, lanelet::Id>>>;

auto matchToLane(
  const Pose & map_pose, const BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance = 1.0,
  const double reduction_ratio = DEFAULT_MATCH_TO_LANE_REDUCTION_RATIO,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type)
  -> std::optional<lanelet::Id>;

auto leftLaneletIds(
  const lanelet::Id lanelet_id, const RoutingGraphType type, const bool include_opposite_direction)
  -> lanelet::Ids;

auto rightLaneletIds(
  const lanelet::Id lanelet_id, const RoutingGraphType type, const bool include_opposite_direction)
  -> lanelet::Ids;
}  // namespace pose
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__LANELET_WRAPPER_POSE_HPP_
