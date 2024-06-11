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

#include <geometry/bounding_box.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_map_core/lanelet_map.hpp>
#include <traffic_simulator/lanelet_map_core/pose.hpp>
#include <traffic_simulator/lanelet_map_core/route.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace pose
{
auto quietNaNPose() -> Pose
{
  return geometry_msgs::build<Pose>()
    .position(geometry_msgs::build<Point>()
                .x(std::numeric_limits<double>::quiet_NaN())
                .y(std::numeric_limits<double>::quiet_NaN())
                .z(std::numeric_limits<double>::quiet_NaN()))
    .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1));
}

auto quietNaNLaneletPose() -> LaneletPose
{
  return traffic_simulator_msgs::build<LaneletPose>()
    .lanelet_id(std::numeric_limits<std::int64_t>::max())
    .s(std::numeric_limits<double>::quiet_NaN())
    .offset(std::numeric_limits<double>::quiet_NaN())
    .rpy(geometry_msgs::build<Vector3>()
           .x(std::numeric_limits<double>::quiet_NaN())
           .y(std::numeric_limits<double>::quiet_NaN())
           .z(std::numeric_limits<double>::quiet_NaN()));
}

auto isInLanelet(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose, const lanelet::Id lanelet_id,
  const double tolerance) -> bool
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{false};
  constexpr bool allow_lane_change{false};

  if (isSameLaneletId(canonicalized_lanelet_pose, lanelet_id)) {
    return true;
  } else {
    const auto start_lanelet_pose = helper::constructCanonicalizedLaneletPose(lanelet_id, 0.0, 0.0);
    if (const auto distance_to_start_lanelet_pose = distance::longitudinalDistance(
          start_lanelet_pose, canonicalized_lanelet_pose, include_adjacent_lanelet,
          include_opposite_direction, allow_lane_change);
        distance_to_start_lanelet_pose and
        std::abs(distance_to_start_lanelet_pose.value()) < tolerance) {
      return true;
    }

    const auto end_lanelet_pose = helper::constructCanonicalizedLaneletPose(
      lanelet_id, lanelet_map_core::lanelet_map::laneletLength(lanelet_id), 0.0);
    if (const auto distance_to_end_lanelet_pose = distance::longitudinalDistance(
          canonicalized_lanelet_pose, end_lanelet_pose, include_adjacent_lanelet,
          include_opposite_direction, allow_lane_change);
        distance_to_end_lanelet_pose and
        std::abs(distance_to_end_lanelet_pose.value()) < tolerance) {
      return true;
    }
  }
  return false;
}

auto isInLanelet(const Point & point, const lanelet::Id lanelet_id) -> bool
{
  return lanelet_map_core::lanelet_map::isInLanelet(point, lanelet_id);
}

auto isAtEndOfLanelets(const CanonicalizedLaneletPose & canonicalized_lanelet_pose) -> bool
{
  const auto lanelet_pose = static_cast<LaneletPose>(canonicalized_lanelet_pose);
  return lanelet_map_core::route::followingLanelets(lanelet_pose.lanelet_id).size() == 1 &&
         lanelet_map_core::lanelet_map::laneletLength(lanelet_pose.lanelet_id) <= lanelet_pose.s;
}

// Conversions
auto toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> Pose
{
  return static_cast<Pose>(lanelet_pose);
}

auto toMapPose(const LaneletPose & lanelet_pose) -> Pose
{
  return lanelet_map_core::pose::toMapPose(
           lanelet_pose, CanonicalizedLaneletPose::getConsiderPoseByRoadSlope())
    .pose;
}

auto canonicalize(const LaneletPose & lanelet_pose) -> LaneletPose
{
  if (
    const auto canonicalized = std::get<std::optional<LaneletPose>>(
      lanelet_map_core::pose::canonicalizeLaneletPose(lanelet_pose))) {
    return canonicalized.value();
  } else {
    THROW_SEMANTIC_ERROR(
      "Lanelet pose (id=", lanelet_pose.lanelet_id, ",s=", lanelet_pose.s,
      ",offset=", lanelet_pose.offset, ",rpy.x=", lanelet_pose.rpy.x, ",rpy.y=", lanelet_pose.rpy.y,
      ",rpy.z=", lanelet_pose.rpy.z, ") is invalid, please check lanelet length and connection.");
  }
}

auto canonicalize(const LaneletPose & lanelet_pose, const lanelet::Ids & route_lanelets)
  -> LaneletPose
{
  if (
    const auto canonicalized = std::get<std::optional<LaneletPose>>(
      lanelet_map_core::pose::canonicalizeLaneletPose(lanelet_pose, route_lanelets))) {
    return canonicalized.value();
  } else {
    THROW_SEMANTIC_ERROR(
      "Lanelet pose (id=", lanelet_pose.lanelet_id, ",s=", lanelet_pose.s,
      ",offset=", lanelet_pose.offset, ",rpy.x=", lanelet_pose.rpy.x, ",rpy.y=", lanelet_pose.rpy.y,
      ",rpy.z=", lanelet_pose.rpy.z,
      ") is invalid, please check lanelet length, connection and entity route.");
  }
}

auto alternativeLaneletPoses(const LaneletPose & lanelet_pose) -> std::vector<LaneletPose>
{
  return lanelet_map_core::pose::alternativeLaneletPoses(lanelet_pose);
}

auto toCanonicalizedLaneletPose(const LaneletPose & lanelet_pose)
  -> std::optional<CanonicalizedLaneletPose>
{
  if (lanelet_pose == LaneletPose()) {
    return std::nullopt;
  } else {
    return CanonicalizedLaneletPose(lanelet_pose);
  }
}

auto toCanonicalizedLaneletPose(const Pose & map_pose, const bool include_crosswalk)
  -> std::optional<CanonicalizedLaneletPose>
{
  /// @todo here matching_distance should be passed
  constexpr double matching_distance{1.0};
  if (
    const auto pose =
      lanelet_map_core::pose::toLaneletPose(map_pose, include_crosswalk, matching_distance)) {
    return toCanonicalizedLaneletPose(pose.value());
  } else {
    return std::nullopt;
  }
}

auto toCanonicalizedLaneletPose(
  const Pose & map_pose, const BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>
{
  if (
    const auto pose = lanelet_map_core::pose::toLaneletPose(
      map_pose, bounding_box, include_crosswalk, matching_distance)) {
    return toCanonicalizedLaneletPose(pose.value());
  } else {
    return std::nullopt;
  }
}

auto toCanonicalizedLaneletPose(
  const Point & map_point, const BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>
{
  return toCanonicalizedLaneletPose(
    geometry_msgs::build<geometry_msgs::msg::Pose>().position(map_point).orientation(
      geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1)),
    bounding_box, include_crosswalk, matching_distance);
}

auto toCanonicalizedLaneletPose(
  const Pose & map_pose, const BoundingBox & bounding_box,
  const lanelet::Ids & unique_route_lanelets, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>
{
  std::optional<LaneletPose> lanelet_pose;
  if (!unique_route_lanelets.empty()) {
    lanelet_pose =
      lanelet_map_core::pose::toLaneletPose(map_pose, unique_route_lanelets, matching_distance);
  }
  if (!lanelet_pose) {
    lanelet_pose = lanelet_map_core::pose::toLaneletPose(
      map_pose, bounding_box, include_crosswalk, matching_distance);
  }
  if (lanelet_pose) {
    return toCanonicalizedLaneletPose(lanelet_pose.value());
  } else {
    return std::nullopt;
  }
}

auto transformRelativePoseToGlobal(const Pose & global_pose, const Pose & relative_pose) -> Pose
{
  tf2::Transform ref_transform, relative_transform;
  tf2::fromMsg(global_pose, ref_transform);
  tf2::fromMsg(relative_pose, relative_transform);
  Pose ret;
  tf2::toMsg(ref_transform * relative_transform, ret);
  return ret;
}

// Relative msg::Pose
auto relativePose(const Pose & from, const Pose & to) -> std::optional<Pose>
{
  try {
    return math::geometry::getRelativePose(from, to);
  } catch (...) {
    return std::nullopt;
  }
}

auto relativePose(const Pose & from, const CanonicalizedLaneletPose & to) -> std::optional<Pose>
{
  return relativePose(from, static_cast<Pose>(to));
}

auto relativePose(const CanonicalizedLaneletPose & from, const Pose & to) -> std::optional<Pose>
{
  return relativePose(static_cast<Pose>(from), to);
}

auto boundingBoxRelativePose(
  const Pose & from, const BoundingBox & from_bounding_box, const Pose & to,
  const BoundingBox & to_bounding_box) -> std::optional<Pose>
{
  if (const auto closest_points =
        math::geometry::getClosestPoses(from, from_bounding_box, to, to_bounding_box);
      closest_points) {
    const auto from_pose_bounding_box = relativePose(from, closest_points.value().first);
    const auto to_pose_bounding_box = relativePose(from, closest_points.value().second);
    if (from_pose_bounding_box && to_pose_bounding_box) {
      return math::geometry::subtractPoses(
        from_pose_bounding_box.value(), to_pose_bounding_box.value());
    }
  }
  return std::nullopt;
}

// Relative LaneletPose
auto relativeLaneletPose(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  const bool allow_lane_change) -> LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  LaneletPose position = quietNaNLaneletPose();
  // here the s and offset are intentionally assigned independently, even if
  // it is not possible to calculate one of them - it happens that one is sufficient
  if (
    const auto longitudinal_distance = distance::longitudinalDistance(
      from, to, include_adjacent_lanelet, include_opposite_direction, allow_lane_change)) {
    position.s = longitudinal_distance.value();
  }
  if (const auto lateral_distance = distance::lateralDistance(from, to, allow_lane_change)) {
    position.offset = lateral_distance.value();
  }
  return position;
}

auto boundingBoxRelativeLaneletPose(
  const CanonicalizedLaneletPose & from, const BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & to, const BoundingBox & to_bounding_box,
  const bool allow_lane_change) -> LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  LaneletPose position = quietNaNLaneletPose();
  // here the s and offset are intentionally assigned independently, even if
  // it is not possible to calculate one of them - it happens that one is sufficient
  if (
    const auto longitudinal_bounding_box_distance = distance::boundingBoxLaneLongitudinalDistance(
      from, from_bounding_box, to, to_bounding_box, include_adjacent_lanelet,
      include_opposite_direction, allow_lane_change)) {
    position.s = longitudinal_bounding_box_distance.value();
  }
  if (
    const auto lateral_bounding_box_distance = distance::boundingBoxLaneLateralDistance(
      from, from_bounding_box, to, to_bounding_box, allow_lane_change)) {
    position.offset = lateral_bounding_box_distance.value();
  }
  return position;
}

/*
  This function has been moved from pedestrian_action_node and modified,
  in case of inconsistency please compare in original:
  https://github.com/tier4/scenario_simulator_v2/blob/090a8d08bcb065d293a530cf641a953edf311f9f/simulation/behavior_tree_plugin/src/pedestrian/pedestrian_action_node.cpp#L67-L128
*/
auto estimateCanonicalizedLaneletPose(
  const Pose & map_pose, const BoundingBox & bounding_box,
  const lanelet::Ids & unique_route_lanelets, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>
{
  if (
    const auto canonicalized_lanelet_pose = toCanonicalizedLaneletPose(
      map_pose, bounding_box, unique_route_lanelets, include_crosswalk, matching_distance)) {
    return canonicalized_lanelet_pose;
  }
  /**
   * @note Hard coded parameter. 2.0 is a matching threshold for lanelet.
   * In this branch, the algorithm only consider entity pose.
   */
  if (
    const auto lanelet_pose =
      lanelet_map_core::pose::toLaneletPose(map_pose, include_crosswalk, 2.0)) {
    const auto canonicalized_tuple =
      lanelet_map_core::pose::canonicalizeLaneletPose(lanelet_pose.value());
    if (
      const auto canonicalized_lanelet_pose =
        std::get<std::optional<LaneletPose>>(canonicalized_tuple)) {
      return toCanonicalizedLaneletPose(lanelet_pose.value());
    } else {
      /// @note If canonicalize failed, set end of road lanelet pose.
      if (
        const auto end_of_road_lanelet_id =
          std::get<std::optional<lanelet::Id>>(canonicalized_tuple)) {
        if (lanelet_pose.value().s < 0) {
          return CanonicalizedLaneletPose(traffic_simulator_msgs::build<LaneletPose>()
                                            .lanelet_id(end_of_road_lanelet_id.value())
                                            .s(0.0)
                                            .offset(lanelet_pose.value().offset)
                                            .rpy(lanelet_pose.value().rpy));
        } else {
          return CanonicalizedLaneletPose(
            traffic_simulator_msgs::build<LaneletPose>()
              .lanelet_id(end_of_road_lanelet_id.value())
              .s(lanelet_map_core::lanelet_map::laneletLength(end_of_road_lanelet_id.value()))
              .offset(lanelet_pose.value().offset)
              .rpy(lanelet_pose.value().rpy));
        }
      } else {
        THROW_SIMULATION_ERROR("Failed to find trailing lanelet_id for LaneletPose estimation.");
      }
    }
  } else {
    return std::nullopt;
  }
}
}  // namespace pose
}  // namespace traffic_simulator
