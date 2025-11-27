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

#include <geometry/bounding_box.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator/utils/route.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
inline namespace pose
{
auto quietNaNPose() -> geometry_msgs::msg::Pose
{
  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(geometry_msgs::build<geometry_msgs::msg::Point>()
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
    .rpy(geometry_msgs::build<geometry_msgs::msg::Vector3>()
           .x(std::numeric_limits<double>::quiet_NaN())
           .y(std::numeric_limits<double>::quiet_NaN())
           .z(std::numeric_limits<double>::quiet_NaN()));
}

/// @note Conversions
auto canonicalize(const LaneletPose & lanelet_pose) -> LaneletPose
{
  if (
    const auto canonicalized = std::get<std::optional<LaneletPose>>(
      lanelet_wrapper::pose::canonicalizeLaneletPose(lanelet_pose))) {
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
      lanelet_wrapper::pose::canonicalizeLaneletPose(lanelet_pose, route_lanelets))) {
    return canonicalized.value();
  } else {
    THROW_SEMANTIC_ERROR(
      "Lanelet pose (id=", lanelet_pose.lanelet_id, ",s=", lanelet_pose.s,
      ",offset=", lanelet_pose.offset, ",rpy.x=", lanelet_pose.rpy.x, ",rpy.y=", lanelet_pose.rpy.y,
      ",rpy.z=", lanelet_pose.rpy.z,
      ") is invalid, please check lanelet length, connection and entity route.");
  }
}

auto toMapPose(const CanonicalizedLaneletPose & lanelet_pose) -> geometry_msgs::msg::Pose
{
  return static_cast<geometry_msgs::msg::Pose>(lanelet_pose);
}

auto toMapPose(const LaneletPose & lanelet_pose) -> geometry_msgs::msg::Pose
{
  return lanelet_wrapper::pose::toMapPose(
           lanelet_pose, CanonicalizedLaneletPose::getConsiderPoseByRoadSlope())
    .pose;
}

auto alternativeLaneletPoses(const LaneletPose & lanelet_pose) -> std::vector<LaneletPose>
{
  return lanelet_wrapper::pose::alternativeLaneletPoses(lanelet_pose);
}

auto toCanonicalizedLaneletPose(const LaneletPose & lanelet_pose)
  -> std::optional<CanonicalizedLaneletPose>
{
  if (lanelet_pose == LaneletPose()) {
    return std::nullopt;
  } else {
    try {
      return CanonicalizedLaneletPose(lanelet_pose);
    } catch (const common::SemanticError &) {
      return std::nullopt;
    }
  }
}

auto toCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose, const bool include_crosswalk)
  -> std::optional<CanonicalizedLaneletPose>
{
  /// @todo here matching_distance should be passed
  constexpr double matching_distance{1.0};
  if (
    const auto pose =
      lanelet_wrapper::pose::toLaneletPose(map_pose, include_crosswalk, matching_distance)) {
    return toCanonicalizedLaneletPose(pose.value());
  } else {
    return std::nullopt;
  }
}

auto toCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>
{
  if (
    const auto pose = lanelet_wrapper::pose::toLaneletPose(
      map_pose, bounding_box, include_crosswalk, matching_distance)) {
    return toCanonicalizedLaneletPose(pose.value());
  } else {
    return std::nullopt;
  }
}

auto toCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box,
  const lanelet::Ids & unique_route_lanelets, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>
{
  std::optional<LaneletPose> lanelet_pose;
  if (!unique_route_lanelets.empty()) {
    lanelet_pose =
      lanelet_wrapper::pose::toLaneletPose(map_pose, unique_route_lanelets, matching_distance);
  }
  if (!lanelet_pose) {
    lanelet_pose = lanelet_wrapper::pose::toLaneletPose(
      map_pose, bounding_box, include_crosswalk, matching_distance);
  }
  if (lanelet_pose) {
    return toCanonicalizedLaneletPose(lanelet_pose.value());
  } else {
    return std::nullopt;
  }
}

auto transformRelativePoseToGlobal(
  const geometry_msgs::msg::Pose & global_pose, const geometry_msgs::msg::Pose & relative_pose)
  -> geometry_msgs::msg::Pose
{
  tf2::Transform ref_transform, relative_transform;
  tf2::fromMsg(global_pose, ref_transform);
  tf2::fromMsg(relative_pose, relative_transform);
  geometry_msgs::msg::Pose ret;
  tf2::toMsg(ref_transform * relative_transform, ret);
  return ret;
}

/// @note This function does not modify the orientation of entity.
auto updatePositionForLaneletTransition(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose, const lanelet::Id next_lanelet_id,
  const geometry_msgs::msg::Vector3 & desired_velocity, const bool desired_velocity_is_global,
  const double step_time) -> std::optional<geometry_msgs::msg::Point>
{
  using math::geometry::operator*;
  using math::geometry::operator+=;

  const auto lanelet_pose = static_cast<LaneletPose>(canonicalized_lanelet_pose);

  /// @note Determine the displacement in the 2D lanelet coordinate system
  Eigen::Vector2d displacement;
  if (desired_velocity_is_global) {
    /// @note Transform desired (global) velocity to local velocity
    const auto map_pose = static_cast<geometry_msgs::msg::Pose>(canonicalized_lanelet_pose);
    const Eigen::Vector3d global_velocity(
      desired_velocity.x, desired_velocity.y, desired_velocity.z);
    const Eigen::Quaterniond quaternion(
      map_pose.orientation.w, map_pose.orientation.x, map_pose.orientation.y,
      map_pose.orientation.z);
    const Eigen::Vector3d local_velocity = quaternion.inverse() * global_velocity;
    displacement = Eigen::Rotation2Dd(lanelet_pose.rpy.z) *
                   Eigen::Vector2d(local_velocity.x(), local_velocity.y()) * step_time;
  } else {
    displacement = Eigen::Rotation2Dd(lanelet_pose.rpy.z) *
                   Eigen::Vector2d(desired_velocity.x, desired_velocity.y) * step_time;
  }
  const auto longitudinal_d = displacement.x();
  const auto lateral_d = displacement.y();

  const auto remaining_lanelet_length =
    lanelet_wrapper::lanelet_map::laneletLength(lanelet_pose.lanelet_id) - lanelet_pose.s;
  const auto next_lanelet_longitudinal_d = longitudinal_d - remaining_lanelet_length;
  if (longitudinal_d <= remaining_lanelet_length) {
    return std::nullopt;
  } else if (  /// @note if longitudinal displacement exceeds the current lanelet length, use next lanelet if possible
    next_lanelet_longitudinal_d < lanelet_wrapper::lanelet_map::laneletLength(next_lanelet_id)) {
    LaneletPose result_lanelet_pose;
    result_lanelet_pose.lanelet_id = next_lanelet_id;
    result_lanelet_pose.s = next_lanelet_longitudinal_d;
    result_lanelet_pose.offset = lanelet_pose.offset + lateral_d;
    result_lanelet_pose.rpy = lanelet_pose.rpy;
    return toMapPose(result_lanelet_pose).position;
  } else {
    THROW_SIMULATION_ERROR(
      "Next lanelet is too short: lanelet_id==", next_lanelet_id, " is shorter than ",
      next_lanelet_longitudinal_d);
  }
}

/// @note Relative msg::Pose
auto isAltitudeMatching(
  const CanonicalizedLaneletPose & lanelet_pose,
  const CanonicalizedLaneletPose & target_lanelet_pose) -> bool
{
  return lanelet_wrapper::pose::isAltitudeWithinThreshold(
    lanelet_pose.getAltitude(), target_lanelet_pose.getAltitude());
}

auto relativePose(const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  try {
    return math::geometry::getRelativePose(from, to);
  } catch (...) {
    return std::nullopt;
  }
}

auto relativePose(const geometry_msgs::msg::Pose & from, const CanonicalizedLaneletPose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  return relativePose(from, static_cast<geometry_msgs::msg::Pose>(to));
}

auto relativePose(const CanonicalizedLaneletPose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  return relativePose(static_cast<geometry_msgs::msg::Pose>(from), to);
}

auto boundingBoxRelativePose(
  const geometry_msgs::msg::Pose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const geometry_msgs::msg::Pose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box)
  -> std::optional<geometry_msgs::msg::Pose>
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

/// @note Relative LaneletPose
/// @todo HdMapUtils will be removed when lanelet_wrapper::distance is added
auto relativeLaneletPose(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  const RoutingConfiguration & routing_configuration) -> LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  /**
   * @note include_opposite_direction value was changed, because of https://github.com/tier4/scenario_simulator_v2/pull/1468
   * (exactly these changes: https://github.com/tier4/scenario_simulator_v2/commit/e6f8c24f614b1a2236b0ee31de51f2a6748e8fec)
   * Calling lanelet_wrapper::pose::leftLaneletIds and rightLaneletIds with
   * include_opposite_direction=true throws error, because it is not implemented yet.
   * Call stack for this to happen is pose::relativeLaneletPose -> distance::longitudinalDistance ->
   * lanelet_wrapper::pose::toLaneletPose -> lanelet_wrapper::pose::leftLaneletIds
   */
  constexpr bool include_opposite_direction{false};

  LaneletPose position = quietNaNLaneletPose();
  /// @note here the s and offset are intentionally assigned independently, even if
  /// it is not possible to calculate one of them - it happens that one is sufficient
  if (
    const auto longitudinal_distance = distance::longitudinalDistance(
      from, to, include_adjacent_lanelet, include_opposite_direction, routing_configuration)) {
    position.s = longitudinal_distance.value();
  }
  if (const auto lateral_distance = distance::lateralDistance(from, to, routing_configuration)) {
    position.offset = lateral_distance.value();
  }
  return position;
}

auto boundingBoxRelativeLaneletPose(
  const CanonicalizedLaneletPose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box,
  const RoutingConfiguration & routing_configuration) -> LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  LaneletPose position = quietNaNLaneletPose();
  /// @note here the s and offset are intentionally assigned independently, even if
  /// it is not possible to calculate one of them - it happens that one is sufficient
  if (
    const auto longitudinal_bounding_box_distance = distance::boundingBoxLaneLongitudinalDistance(
      from, from_bounding_box, to, to_bounding_box, include_adjacent_lanelet,
      include_opposite_direction, routing_configuration)) {
    position.s = longitudinal_bounding_box_distance.value();
  }
  if (
    const auto lateral_bounding_box_distance = distance::boundingBoxLaneLateralDistance(
      from, from_bounding_box, to, to_bounding_box, routing_configuration)) {
    position.offset = lateral_bounding_box_distance.value();
  }
  return position;
}

auto isInLanelet(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose, const lanelet::Id lanelet_id,
  const double tolerance) -> bool
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{false};
  constexpr RoutingConfiguration routing_configuration;

  if (isSameLaneletId(canonicalized_lanelet_pose, lanelet_id)) {
    return true;
  } else {
    const auto start_lanelet_pose = helper::constructCanonicalizedLaneletPose(lanelet_id, 0.0, 0.0);
    if (const auto distance_to_start_lanelet_pose = distance::longitudinalDistance(
          start_lanelet_pose, canonicalized_lanelet_pose, include_adjacent_lanelet,
          include_opposite_direction, routing_configuration);
        distance_to_start_lanelet_pose and
        std::abs(distance_to_start_lanelet_pose.value()) <= tolerance) {
      return true;
    }

    const auto end_lanelet_pose = helper::constructCanonicalizedLaneletPose(
      lanelet_id, lanelet_wrapper::lanelet_map::laneletLength(lanelet_id), 0.0);
    if (const auto distance_to_end_lanelet_pose = distance::longitudinalDistance(
          canonicalized_lanelet_pose, end_lanelet_pose, include_adjacent_lanelet,
          include_opposite_direction, routing_configuration);
        distance_to_end_lanelet_pose and
        std::abs(distance_to_end_lanelet_pose.value()) <= tolerance) {
      return true;
    }
  }
  return false;
}

auto isInLanelet(const geometry_msgs::msg::Point & point, const lanelet::Id lanelet_id) -> bool
{
  return lanelet_wrapper::lanelet_map::isInLanelet(lanelet_id, point);
}

/// @todo passing HdMapUtils will be removed when lanelet_wrapper::route::followingLanelets is added
auto isAtEndOfLanelets(
  const CanonicalizedLaneletPose & canonicalized_lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> bool
{
  const auto lanelet_pose = static_cast<LaneletPose>(canonicalized_lanelet_pose);
  return hdmap_utils_ptr->getFollowingLanelets(lanelet_pose.lanelet_id).size() == 1 &&
         lanelet_wrapper::lanelet_map::laneletLength(lanelet_pose.lanelet_id) <= lanelet_pose.s;
}

auto findRoutableAlternativeLaneletPoseFrom(
  const lanelet::Id from_lanelet_id, const CanonicalizedLaneletPose & to_canonicalized_lanelet_pose,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box)
  -> std::optional<traffic_simulator::CanonicalizedLaneletPose>
{
  /// @note search_distance should be minimal, just to check nearest neighbour lanelets
  constexpr double search_distance{3.0};
  constexpr bool include_crosswalk{false};
  /**
   * @note route::route requires routing_configuration,
   * 'allow_lane_change = true' is needed to check distances to entities on neighbour lanelets
   */
  RoutingConfiguration routing_configuration;
  routing_configuration.allow_lane_change = true;

  /// @note if there is already a route from_lanelet_id->to_lanelet_id, return it
  /// if not, transform the 'to_lanelet_id' position into the nearby lanelets and search for a route in relation to them
  if (const auto to_lanelet_id = to_canonicalized_lanelet_pose.getLaneletPose().lanelet_id;
      !route::route(from_lanelet_id, to_lanelet_id, routing_configuration).empty()) {
    return to_canonicalized_lanelet_pose;
  } else if (const auto nearby_lanelet_ids = lanelet_wrapper::pose::findMatchingLanes(
               static_cast<geometry_msgs::msg::Pose>(to_canonicalized_lanelet_pose),
               to_bounding_box, include_crosswalk, search_distance,
               lanelet_wrapper::pose::DEFAULT_MATCH_TO_LANE_REDUCTION_RATIO,
               routing_configuration.routing_graph_type);
             nearby_lanelet_ids.has_value()) {
    std::vector<std::pair<CanonicalizedLaneletPose, lanelet::Ids>> routes;
    for (const auto & [distance, lanelet_id] : nearby_lanelet_ids.value()) {
      if (auto route = route::route(from_lanelet_id, lanelet_id, routing_configuration);
          lanelet_id == to_lanelet_id || route.empty()) {
        continue;
      } else if (const auto lanelet_pose = lanelet_wrapper::pose::toLaneletPose(
                   static_cast<geometry_msgs::msg::Pose>(to_canonicalized_lanelet_pose), lanelet_id,
                   search_distance);
                 !lanelet_pose) {
        continue;
      } else if (auto canonicalized = toCanonicalizedLaneletPose(lanelet_pose.value());
                 canonicalized) {
        routes.emplace_back(std::move(canonicalized.value()), std::move(route));
      }
    }
    if (!routes.empty()) {
      /// @note we want to have alternative lanelet pose with shortest route so we search for minimum
      return std::min_element(
               routes.cbegin(), routes.cend(),
               [](const auto & a, const auto & b) { return a.second.size() < b.second.size(); })
        ->first;
    } else {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

namespace pedestrian
{
/*
  This function has been moved from pedestrian_action_node and modified,
  in case of inconsistency please compare in original:
  https://github.com/tier4/scenario_simulator_v2/blob/090a8d08bcb065d293a530cf641a953edf311f9f/simulation/behavior_tree_plugin/src/pedestrian/pedestrian_action_node.cpp#L67-L128
*/
auto transformToCanonicalizedLaneletPose(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box,
  const lanelet::Ids & unique_route_lanelets, const bool include_crosswalk,
  const double matching_distance) -> std::optional<CanonicalizedLaneletPose>
{
  if (
    const auto canonicalized_lanelet_pose = toCanonicalizedLaneletPose(
      map_pose, bounding_box, unique_route_lanelets, include_crosswalk, matching_distance)) {
    return canonicalized_lanelet_pose;
  }
  /// @note Hard coded parameter. 2.0 is a matching threshold for lanelet.
  /// In this branch, the algorithm only consider entity pose.
  if (
    const auto lanelet_pose =
      lanelet_wrapper::pose::toLaneletPose(map_pose, include_crosswalk, 2.0)) {
    const auto canonicalized_tuple =
      lanelet_wrapper::pose::canonicalizeLaneletPose(lanelet_pose.value());
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
              .s(lanelet_wrapper::lanelet_map::laneletLength(end_of_road_lanelet_id.value()))
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
}  // namespace pedestrian
}  // namespace pose
}  // namespace traffic_simulator
