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
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/operator.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/inner_product.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace pose
{
auto toMapPose(const LaneletPose & lanelet_pose, const bool fill_pitch) -> PoseStamped
{
  using math::geometry::operator*;
  using math::geometry::operator+=;
  if (
    const auto canonicalized_lanelet_pose =
      std::get<std::optional<LaneletPose>>(pose::canonicalizeLaneletPose(lanelet_pose))) {
    PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    const auto lanelet_spline =
      lanelet_map::centerPointsSpline(canonicalized_lanelet_pose->lanelet_id);
    /// @note map position
    pose_stamped.pose = lanelet_spline->getPose(canonicalized_lanelet_pose->s);
    pose_stamped.pose.position +=
      math::geometry::normalize(lanelet_spline->getNormalVector(canonicalized_lanelet_pose->s)) *
      canonicalized_lanelet_pose->offset;
    /// @note map orientation
    const auto tangent_vector = lanelet_spline->getTangentVector(canonicalized_lanelet_pose->s);
    const auto lanelet_rpy =
      geometry_msgs::build<Vector3>()
        .x(0.0)
        .y(
          fill_pitch ? std::atan2(-tangent_vector.z, std::hypot(tangent_vector.x, tangent_vector.y))
                     : 0.0)
        .z(std::atan2(tangent_vector.y, tangent_vector.x));
    pose_stamped.pose.orientation =
      math::geometry::convertEulerAngleToQuaternion(lanelet_rpy) *
      math::geometry::convertEulerAngleToQuaternion(canonicalized_lanelet_pose->rpy);
    return pose_stamped;
  } else {
    THROW_SEMANTIC_ERROR(
      "Lanelet pose (id=", lanelet_pose.lanelet_id, ",s=", lanelet_pose.s,
      ",offset=", lanelet_pose.offset, ",rpy.x=", lanelet_pose.rpy.x, ",rpy.y=", lanelet_pose.rpy.y,
      ",rpy.z=", lanelet_pose.rpy.z, ") is invalid, please check lanelet length and connection.");
  }
}

auto isAltitudeMatching(const double current_altitude, const double target_altitude) -> bool
{
  return std::abs(current_altitude - target_altitude) <= ALTITUDE_THRESHOLD;
}

auto toLaneletPose(
  const Pose & map_pose, const lanelet::Id lanelet_id, const double matching_distance)
  -> std::optional<LaneletPose>
{
  /// @note yaw_threshold_deg is used to determine whether the entity is going straight,
  /// it defines the maximum allowed rotation with respect to the lanelet centerline.
  constexpr double yaw_threshold_deg = 45.0;

  const auto lanelet_spline = lanelet_map::centerPointsSpline(lanelet_id);
  if (const auto lanelet_pose_s = lanelet_spline->getSValue(map_pose, matching_distance);
      !lanelet_pose_s) {
    return std::nullopt;
  } else if (const auto pose_on_centerline = lanelet_spline->getPose(lanelet_pose_s.value());
             !isAltitudeMatching(map_pose.position.z, pose_on_centerline.position.z)) {
    return std::nullopt;
  } else {
    constexpr double yaw_range_min_rad = M_PI * yaw_threshold_deg / 180.0;
    constexpr double yaw_range_max_rad = M_PI - yaw_range_min_rad;
    if (const auto lanelet_pose_rpy = math::geometry::convertQuaternionToEulerAngle(
          math::geometry::getRotation(pose_on_centerline.orientation, map_pose.orientation));
        std::fabs(lanelet_pose_rpy.z) > yaw_range_min_rad and
        std::fabs(lanelet_pose_rpy.z) < yaw_range_max_rad) {
      return std::nullopt;
    } else {
      double lanelet_pose_offset = std::sqrt(
        lanelet_spline->getSquaredDistanceIn2D(map_pose.position, lanelet_pose_s.value()));
      if (const double inner_product = math::geometry::innerProduct(
            lanelet_spline->getNormalVector(lanelet_pose_s.value()),
            lanelet_spline->getSquaredDistanceVector(map_pose.position, lanelet_pose_s.value()));
          inner_product < 0) {
        lanelet_pose_offset = lanelet_pose_offset * -1;
      }
      return traffic_simulator_msgs::build<LaneletPose>()
        .lanelet_id(lanelet_id)
        .s(lanelet_pose_s.value())
        .offset(lanelet_pose_offset)
        .rpy(lanelet_pose_rpy);
    }
  }
}

auto toLaneletPose(
  const Pose & map_pose, const lanelet::Ids & lanelet_ids, const double matching_distance)
  -> std::optional<LaneletPose>
{
  for (const auto & lanelet_id : lanelet_ids) {
    if (const auto lanelet_pose = toLaneletPose(map_pose, lanelet_id, matching_distance);
        lanelet_pose) {
      return lanelet_pose.value();
    }
  }
  return std::nullopt;
}

auto toLaneletPose(
  const Pose & map_pose, const bool include_crosswalk, const double matching_distance)
  -> std::optional<LaneletPose>
{
  /// @note Hardcoded parameter, this value has no technical basis and is determined based on experimentation.
  /// @todo Add doxygen comments as soon as you know the meaning and rationale of the value.
  constexpr double distance_threshold{0.1};
  constexpr std::size_t search_count{5};
  const auto nearby_lanelet_ids = lanelet_map::nearbyLaneletIds(
    map_pose.position, distance_threshold, include_crosswalk, search_count);
  return toLaneletPose(map_pose, nearby_lanelet_ids, matching_distance);
}

auto toLaneletPose(
  const Pose & map_pose, const BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance, const RoutingGraphType type) -> std::optional<LaneletPose>
{
  const auto lanelet_id_using_bounding_box = matchToLane(
    map_pose, bounding_box, include_crosswalk, matching_distance,
    DEFAULT_MATCH_TO_LANE_REDUCTION_RATIO, type);

  if (!lanelet_id_using_bounding_box) {
    return toLaneletPose(map_pose, include_crosswalk, matching_distance);
  } else if (
    const auto lanelet_pose_using_bounding_box =
      toLaneletPose(map_pose, lanelet_id_using_bounding_box.value(), matching_distance)) {
    return lanelet_pose_using_bounding_box;
  }

  for (const auto & previous_lanelet_id :
       lanelet_map::previousLaneletIds(lanelet_id_using_bounding_box.value(), type)) {
    if (
      const auto lanelet_pose_in_previous_lanelet =
        toLaneletPose(map_pose, previous_lanelet_id, matching_distance)) {
      return lanelet_pose_in_previous_lanelet;
    }
  }

  for (const auto & next_lanelet_id :
       lanelet_map::nextLaneletIds(lanelet_id_using_bounding_box.value(), type)) {
    if (
      const auto lanelet_pose_in_next_lanelet =
        toLaneletPose(map_pose, next_lanelet_id, matching_distance)) {
      return lanelet_pose_in_next_lanelet;
    }
  }
  return toLaneletPose(map_pose, include_crosswalk, matching_distance);
}

auto toLaneletPoses(
  const Pose & map_pose, const lanelet::Id lanelet_id, const double matching_distance,
  const bool include_opposite_direction, const RoutingGraphType type) -> std::vector<LaneletPose>
{
  std::vector<LaneletPose> lanelet_poses;
  std::set<lanelet::Id> new_lanelet_ids_set{lanelet_id};
  auto insertNewIds = [&](const auto & new_ids) {
    new_lanelet_ids_set.insert(new_ids.begin(), new_ids.end());
  };
  insertNewIds(leftLaneletIds(lanelet_id, type, include_opposite_direction));
  insertNewIds(rightLaneletIds(lanelet_id, type, include_opposite_direction));
  insertNewIds(lanelet_map::previousLaneletIds({lanelet_id}, type));
  insertNewIds(lanelet_map::nextLaneletIds({lanelet_id}, type));
  for (const auto & new_lanelet_id : new_lanelet_ids_set) {
    if (const auto & lanelet_pose = toLaneletPose(map_pose, new_lanelet_id, matching_distance)) {
      lanelet_poses.emplace_back(lanelet_pose.value());
    }
  }
  return lanelet_poses;
}

auto alternativeLaneletPoses(const LaneletPose & reference_lanelet_pose) -> std::vector<LaneletPose>
{
  const auto alternativesInPreviousLanelet = [](const auto & lanelet_pose) {
    std::vector<LaneletPose> lanelet_poses_in_previous_lanelet;
    for (const auto & previous_lanelet_id :
         lanelet_map::previousLaneletIds(lanelet_pose.lanelet_id)) {
      const auto lanelet_pose_in_previous_lanelet = helper::constructLaneletPose(
        previous_lanelet_id, lanelet_pose.s + lanelet_map::laneletLength(previous_lanelet_id),
        lanelet_pose.offset);
      if (const auto recursive_alternative_poses =
            alternativeLaneletPoses(lanelet_pose_in_previous_lanelet);
          recursive_alternative_poses.empty()) {
        lanelet_poses_in_previous_lanelet.emplace_back(lanelet_pose_in_previous_lanelet);
      } else {
        lanelet_poses_in_previous_lanelet.insert(
          lanelet_poses_in_previous_lanelet.end(), recursive_alternative_poses.begin(),
          recursive_alternative_poses.end());
      }
    }
    return lanelet_poses_in_previous_lanelet;
  };

  const auto alternativesInNextLanelet = [](const auto & lanelet_pose) {
    std::vector<LaneletPose> lanelet_poses_in_next_lanelet;
    for (const auto & next_lanelet_id : lanelet_map::nextLaneletIds(lanelet_pose.lanelet_id)) {
      const auto lanelet_pose_in_next_lanelet = helper::constructLaneletPose(
        next_lanelet_id, lanelet_pose.s - lanelet_map::laneletLength(lanelet_pose.lanelet_id),
        lanelet_pose.offset);
      if (const auto recursive_alternative_poses =
            alternativeLaneletPoses(lanelet_pose_in_next_lanelet);
          recursive_alternative_poses.empty()) {
        lanelet_poses_in_next_lanelet.emplace_back(lanelet_pose_in_next_lanelet);
      } else {
        lanelet_poses_in_next_lanelet.insert(
          lanelet_poses_in_next_lanelet.end(), recursive_alternative_poses.begin(),
          recursive_alternative_poses.end());
      }
    }
    return lanelet_poses_in_next_lanelet;
  };

  /// @note If s value under 0, it means this pose is on the previous lanelet.
  if (reference_lanelet_pose.s < 0) {
    return alternativesInPreviousLanelet(reference_lanelet_pose);
  }
  /// @note If s value overs it's lanelet length, it means this pose is on the next lanelet.
  else if (
    reference_lanelet_pose.s > (lanelet_map::laneletLength(reference_lanelet_pose.lanelet_id))) {
    return alternativesInNextLanelet(reference_lanelet_pose);
  }
  /// @note If s value is in range [0,length_of_the_lanelet], return lanelet_pose.
  else {
    return {reference_lanelet_pose};
  }
}

auto alongLaneletPose(
  const LaneletPose & from_pose, const double distance, const RoutingGraphType type) -> LaneletPose
{
  auto lanelet_pose = from_pose;
  lanelet_pose.s += distance;
  if (lanelet_pose.s >= 0) {
    while (lanelet_pose.s >= lanelet_map::laneletLength(lanelet_pose.lanelet_id)) {
      auto next_lanelet_ids =
        lanelet_map::nextLaneletIds(lanelet_pose.lanelet_id, "straight", type);
      if (next_lanelet_ids.empty()) {
        /// @note if empty try to use other than "straight", but the first found
        next_lanelet_ids = lanelet_map::nextLaneletIds(lanelet_pose.lanelet_id, type);
      }
      if (next_lanelet_ids.empty()) {
        THROW_SEMANTIC_ERROR(
          "failed to calculate along pose (id,s) = (", from_pose.lanelet_id, ",",
          from_pose.s + distance, "), next lanelet of id = ", lanelet_pose.lanelet_id, "is empty.");
      }
      lanelet_pose.s = lanelet_pose.s - lanelet_map::laneletLength(lanelet_pose.lanelet_id);
      lanelet_pose.lanelet_id = next_lanelet_ids[0];
    }
  } else {
    while (lanelet_pose.s < 0) {
      auto previous_lanelet_ids =
        lanelet_map::previousLaneletIds(lanelet_pose.lanelet_id, "straight", type);
      if (previous_lanelet_ids.empty()) {
        /// @note if empty try to use other than "straight", but the first found
        previous_lanelet_ids = lanelet_map::previousLaneletIds(lanelet_pose.lanelet_id, type);
      }
      if (previous_lanelet_ids.empty()) {
        THROW_SEMANTIC_ERROR(
          "failed to calculate along pose (id,s) = (", from_pose.lanelet_id, ",",
          from_pose.s + distance, "), next lanelet of id = ", lanelet_pose.lanelet_id, "is empty.");
      }
      lanelet_pose.s = lanelet_pose.s + lanelet_map::laneletLength(previous_lanelet_ids[0]);
      lanelet_pose.lanelet_id = previous_lanelet_ids[0];
    }
  }
  return lanelet_pose;
}

auto alongLaneletPose(
  const LaneletPose & from_pose, const lanelet::Ids & route_lanelets, const double distance)
  -> LaneletPose
{
  auto lanelet_pose = from_pose;
  lanelet_pose.s += distance;
  const auto canonicalized = canonicalizeLaneletPose(lanelet_pose, route_lanelets);
  if (
    const auto & canonicalized_lanelet_pose = std::get<std::optional<LaneletPose>>(canonicalized)) {
    /// @note If canonicalize succeed, just return canonicalized pose
    return canonicalized_lanelet_pose.value();
  } else {
    /// @note If canonicalize failed, return lanelet pose as end of road
    if (const auto end_of_road_lanelet_id = std::get<std::optional<lanelet::Id>>(canonicalized)) {
      return traffic_simulator_msgs::build<LaneletPose>()
        .lanelet_id(end_of_road_lanelet_id.value())
        .s(lanelet_pose.s <= 0 ? 0 : lanelet_map::laneletLength(end_of_road_lanelet_id.value()))
        .offset(lanelet_pose.offset)
        .rpy(lanelet_pose.rpy);
    } else {
      THROW_SIMULATION_ERROR("Failed to find trailing lanelet_id.");
    }
  }
}

/**
 * @brief Canonicalizes a given LaneletPose by adjusting the longitudinal position (s) to ensure it
 *        lies within the bounds of the specified lanelet. If the position is out of bounds, it
 *        traverses to previous or next lanelets to find the canonicalized position.
 *
 * If the provided pose has a longitudinal position (s) less than 0, the function traverses
 * to previous lanelets until a valid position is found or no previous lanelets exist.
 *
 * If the longitudinal position (s) exceeds the length of the current lanelet, the function traverses
 * to next lanelets until the position is valid or no next lanelets exist.
 *
 * @param lanelet_pose The input LaneletPose to canonicalize, containing a lanelet ID and longitudinal position (s).
 * @return A tuple where:
 *         - The first element is an optional canonicalized LaneletPose (std::nullopt if canonicalization fails).
 *         - The second element is an optional lanelet ID where the process stopped (std::nullopt if successful).
 */
auto canonicalizeLaneletPose(const LaneletPose & lanelet_pose)
  -> std::tuple<std::optional<LaneletPose>, std::optional<lanelet::Id>>
{
  auto canonicalized_lanelet_pose = lanelet_pose;
  while (canonicalized_lanelet_pose.s < 0) {
    if (const auto previous_lanelet_ids =
          lanelet_map::previousLaneletIds(canonicalized_lanelet_pose.lanelet_id);
        previous_lanelet_ids.empty()) {
      return {std::nullopt, canonicalized_lanelet_pose.lanelet_id};
    } else {
      canonicalized_lanelet_pose.s += lanelet_map::laneletLength(previous_lanelet_ids[0]);
      canonicalized_lanelet_pose.lanelet_id = previous_lanelet_ids[0];
    }
  }
  while (canonicalized_lanelet_pose.s >
         lanelet_map::laneletLength(canonicalized_lanelet_pose.lanelet_id)) {
    if (const auto next_lanelet_ids =
          lanelet_map::nextLaneletIds(canonicalized_lanelet_pose.lanelet_id);
        next_lanelet_ids.empty()) {
      return {std::nullopt, canonicalized_lanelet_pose.lanelet_id};
    } else {
      canonicalized_lanelet_pose.s -=
        lanelet_map::laneletLength(canonicalized_lanelet_pose.lanelet_id);
      canonicalized_lanelet_pose.lanelet_id = next_lanelet_ids[0];
    }
  }
  return {canonicalized_lanelet_pose, std::nullopt};
}

auto canonicalizeLaneletPose(const LaneletPose & lanelet_pose, const lanelet::Ids & route_lanelets)
  -> std::tuple<std::optional<LaneletPose>, std::optional<lanelet::Id>>
{
  if (lanelet_pose.s < 0) {
    /// @note When canonicalizing to backward lanelet_id, do not consider route
    return canonicalizeLaneletPose(lanelet_pose);
  }
  auto canonicalized_lanelet_pose = lanelet_pose;
  while (canonicalized_lanelet_pose.s >
         lanelet_map::laneletLength(canonicalized_lanelet_pose.lanelet_id)) {
    /// @note When canonicalizing to forward lanelet_id, consider route
    bool found_next_lanelet_in_route = false;
    for (const auto & next_lanelet_id :
         lanelet_map::nextLaneletIds(canonicalized_lanelet_pose.lanelet_id)) {
      if (
        std::find(route_lanelets.begin(), route_lanelets.end(), next_lanelet_id) !=
        route_lanelets.end()) {
        found_next_lanelet_in_route = true;
        canonicalized_lanelet_pose.s -=
          lanelet_map::laneletLength(canonicalized_lanelet_pose.lanelet_id);
        canonicalized_lanelet_pose.lanelet_id = next_lanelet_id;
      }
    }
    if (!found_next_lanelet_in_route) {
      return {std::nullopt, canonicalized_lanelet_pose.lanelet_id};
    }
  }
  return {canonicalized_lanelet_pose, std::nullopt};
}

auto findMatchingLanes(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance, const double reduction_ratio,
  const traffic_simulator::RoutingGraphType type)
  -> std::optional<std::set<std::pair<double, lanelet::Id>>>
{
  const auto absoluteHullPolygon = [&reduction_ratio, &bounding_box](const auto & pose) {
    auto relative_hull = lanelet::matching::Hull2d{
      lanelet::BasicPoint2d{
        bounding_box.center.x + bounding_box.dimensions.x * 0.5 * reduction_ratio,
        bounding_box.center.y + bounding_box.dimensions.y * 0.5 * reduction_ratio},
      lanelet::BasicPoint2d{
        bounding_box.center.x - bounding_box.dimensions.x * 0.5 * reduction_ratio,
        bounding_box.center.y - bounding_box.dimensions.y * 0.5 * reduction_ratio}};
    lanelet::BasicPolygon2d absolute_hull_polygon;
    absolute_hull_polygon.reserve(relative_hull.size());
    for (const auto & relative_hull_point : relative_hull) {
      absolute_hull_polygon.push_back(pose * relative_hull_point);
    }
    return absolute_hull_polygon;
  };
  // prepare object for matching
  const auto yaw = math::geometry::convertQuaternionToEulerAngle(map_pose.orientation).z;
  lanelet::matching::Object2d bounding_box_object;
  bounding_box_object.pose.translation() =
    lanelet::BasicPoint2d(map_pose.position.x, map_pose.position.y);
  bounding_box_object.pose.linear() = Eigen::Rotation2D<double>(yaw).matrix();
  bounding_box_object.absoluteHull = absoluteHullPolygon(bounding_box_object.pose);
  // find matches and optionally filter
  auto matches = lanelet::matching::getDeterministicMatches(
    *LaneletWrapper::map(), bounding_box_object, matching_distance);
  if (!include_crosswalk) {
    matches =
      lanelet::matching::removeNonRuleCompliantMatches(matches, LaneletWrapper::trafficRules(type));
  }
  if (!matches.empty()) {
    std::set<std::pair<double, lanelet::Id>> distances_with_ids;
    for (const auto & match : matches) {
      if (
        const auto lanelet_pose = toLaneletPose(map_pose, match.lanelet.id(), matching_distance)) {
        distances_with_ids.emplace(lanelet_pose->offset, lanelet_pose->lanelet_id);
      }
    }
    if (!distances_with_ids.empty()) {
      return distances_with_ids;
    }
  }
  return std::nullopt;
}

auto matchToLane(
  const geometry_msgs::msg::Pose & pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
  const bool include_crosswalk, const double matching_distance, const double reduction_ratio,
  const traffic_simulator::RoutingGraphType type) -> std::optional<lanelet::Id>
{
  /// @note findMatchingLanes returns a container sorted by distance - increasing, but not absolute value
  if (
    const auto matching_lanes =
      findMatchingLanes(pose, bbox, include_crosswalk, matching_distance, reduction_ratio, type)) {
    std::set<std::pair<double, lanelet::Id>> sorted_lanes;
    for (const auto & lane : *matching_lanes) {
      sorted_lanes.insert(std::make_pair(std::abs(lane.first), lane.second));
    }
    return sorted_lanes.begin()->second;
  } else {
    return std::nullopt;
  }
}

auto leftLaneletIds(
  const lanelet::Id lanelet_id, const RoutingGraphType type, const bool include_opposite_direction)
  -> lanelet::Ids
{
  if (include_opposite_direction) {
    throw common::Error(
      "lanelet_wrapper::pose::leftLaneletIds with include_opposite_direction=true is not "
      "implemented yet.");
  } else {
    return lanelet_map::laneletIds(LaneletWrapper::routingGraph(type)->lefts(
      LaneletWrapper::map()->laneletLayer.get(lanelet_id)));
  }
}

auto rightLaneletIds(
  const lanelet::Id lanelet_id, const RoutingGraphType type, const bool include_opposite_direction)
  -> lanelet::Ids
{
  if (include_opposite_direction) {
    throw common::Error(
      "lanelet_wrapper::pose::rightLaneletIds with include_opposite_direction=true is not "
      "implemented "
      "yet.");
  } else {
    return lanelet_map::laneletIds(LaneletWrapper::routingGraph(type)->rights(
      LaneletWrapper::map()->laneletLayer.get(lanelet_id)));
  }
}
}  // namespace pose
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
