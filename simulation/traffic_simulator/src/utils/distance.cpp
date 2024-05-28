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
#include <geometry/distance.hpp>
#include <geometry/transform.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/lanelet_core/distance.hpp>
#include <traffic_simulator/utils/lanelet_core/other.hpp>
#include <traffic_simulator/utils/lanelet_core/pose.hpp>
#include <traffic_simulator/utils/lanelet_core/route.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>

namespace traffic_simulator
{
namespace distance
{
auto lateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  const bool allow_lane_change) -> std::optional<double>
{
  return lanelet_core::distance::getLateralDistance(
    static_cast<LaneletPose>(from), static_cast<LaneletPose>(to), allow_lane_change);
}

auto lateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  const double matching_distance, const bool allow_lane_change) -> std::optional<double>
{
  if (
    std::abs(static_cast<LaneletPose>(from).offset) <= matching_distance &&
    std::abs(static_cast<LaneletPose>(to).offset) <= matching_distance) {
    return lateralDistance(from, to, allow_lane_change);
  } else {
    return std::nullopt;
  }
}

auto longitudinalDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool const include_adjacent_lanelet, bool const include_opposite_direction,
  const bool allow_lane_change) -> std::optional<double>
{
  if (!include_adjacent_lanelet) {
    auto to_canonicalized = static_cast<LaneletPose>(to);
    if (to.hasAlternativeLaneletPose()) {
      if (
        const auto to_canonicalized_opt = to.getAlternativeLaneletPoseBaseOnShortestRouteFrom(
          static_cast<LaneletPose>(from), allow_lane_change)) {
        to_canonicalized = to_canonicalized_opt.value();
      }
    }

    const auto forward_distance = lanelet_core::distance::getLongitudinalDistance(
      static_cast<LaneletPose>(from), to_canonicalized, allow_lane_change);

    const auto backward_distance = lanelet_core::distance::getLongitudinalDistance(
      to_canonicalized, static_cast<LaneletPose>(from), allow_lane_change);

    if (forward_distance && backward_distance) {
      return forward_distance.value() > backward_distance.value() ? -backward_distance.value()
                                                                  : forward_distance.value();
    } else if (forward_distance) {
      return forward_distance.value();
    } else if (backward_distance) {
      return -backward_distance.value();
    } else {
      return std::nullopt;
    }
  } else {
    /// @todo here matching_distance should be passed
    constexpr double matching_distance = 5.0;
    /**
     * @brief hard coded parameter!! 5.0 is a matching distance of the toLaneletPoses function.
     * A matching distance of about 1.5 lane widths is given as the matching distance to match the
     * Entity present on the adjacent Lanelet.
     */
    auto from_poses = lanelet_core::pose::toLaneletPoses(
      static_cast<geometry_msgs::msg::Pose>(from), static_cast<LaneletPose>(from).lanelet_id,
      matching_distance, include_opposite_direction);
    from_poses.emplace_back(from);
    /**
     * @brief hard coded parameter!! 5.0 is a matching distance of the toLaneletPoses function.
     * A matching distance of about 1.5 lane widths is given as the matching distance to match the
     * Entity present on the adjacent Lanelet.
     */
    auto to_poses = lanelet_core::pose::toLaneletPoses(
      static_cast<geometry_msgs::msg::Pose>(to), static_cast<LaneletPose>(to).lanelet_id,
      matching_distance, include_opposite_direction);
    to_poses.emplace_back(to);

    std::vector<double> distances = {};
    for (const auto & from_pose : from_poses) {
      for (const auto & to_pose : to_poses) {
        if (
          const auto distance = longitudinalDistance(
            CanonicalizedLaneletPose(from_pose), CanonicalizedLaneletPose(to_pose), false,
            include_opposite_direction, allow_lane_change)) {
          distances.emplace_back(distance.value());
        }
      }
    }

    if (!distances.empty()) {
      return *std::min_element(distances.begin(), distances.end(), [](double a, double b) {
        return std::abs(a) < std::abs(b);
      });
    } else {
      return std::nullopt;
    }
  }
}

auto boundingBoxDistance(
  const geometry_msgs::msg::Pose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const geometry_msgs::msg::Pose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box) -> std::optional<double>
{
  return math::geometry::getPolygonDistance(from, from_bounding_box, to, to_bounding_box);
}

auto boundingBoxLaneLateralDistance(
  const CanonicalizedLaneletPose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box, const bool allow_lane_change)
  -> std::optional<double>
{
  if (const auto lateral_distance = lateralDistance(from, to, allow_lane_change);
      lateral_distance) {
    const auto from_bounding_box_distances =
      math::geometry::getDistancesFromCenterToEdge(from_bounding_box);
    const auto to_bounding_box_distances =
      math::geometry::getDistancesFromCenterToEdge(to_bounding_box);
    auto bounding_box_distance = 0.0;
    if (lateral_distance.value() > 0.0) {
      bounding_box_distance =
        -std::abs(from_bounding_box_distances.right) - std::abs(to_bounding_box_distances.left);
    } else if (lateral_distance.value() < 0.0) {
      bounding_box_distance =
        std::abs(from_bounding_box_distances.left) + std::abs(to_bounding_box_distances.right);
    }
    return lateral_distance.value() + bounding_box_distance;
  }
  return std::nullopt;
}

auto boundingBoxLaneLongitudinalDistance(
  const CanonicalizedLaneletPose & from,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & to,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box,
  const bool include_adjacent_lanelet, const bool include_opposite_direction,
  const bool allow_lane_change) -> std::optional<double>
{
  if (const auto longitudinal_distance = longitudinalDistance(
        from, to, include_adjacent_lanelet, include_opposite_direction, allow_lane_change);
      longitudinal_distance) {
    const auto from_bounding_box_distances =
      math::geometry::getDistancesFromCenterToEdge(from_bounding_box);
    const auto to_bounding_box_distances =
      math::geometry::getDistancesFromCenterToEdge(to_bounding_box);
    auto bounding_box_distance = 0.0;
    if (longitudinal_distance.value() > 0.0) {
      bounding_box_distance =
        -std::abs(from_bounding_box_distances.front) - std::abs(to_bounding_box_distances.rear);
    } else if (longitudinal_distance.value() < 0.0) {
      bounding_box_distance =
        +std::abs(from_bounding_box_distances.rear) + std::abs(to_bounding_box_distances.front);
    }
    return longitudinal_distance.value() + bounding_box_distance;
  }
  return std::nullopt;
}

auto distanceToLeftLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Id lanelet_id)
  -> double
{
  if (const auto bound = lanelet_core::other::getLeftBound(lanelet_id); bound.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate left bounds of lanelet_id : ", lanelet_id, " please check lanelet map.");
  } else if (const auto polygon =
               math::geometry::transformPoints(map_pose, math::geometry::toPolygon2D(bounding_box));
             polygon.empty()) {
    THROW_SEMANTIC_ERROR("Failed to calculate 2d polygon.");
  } else {
    return math::geometry::getDistance2D(bound, polygon);
  }
}

auto distanceToLeftLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids)
  -> double
{
  std::vector<double> distances;
  std::transform(
    lanelet_ids.begin(), lanelet_ids.end(), std::back_inserter(distances),
    [&](auto lanelet_id) { return distanceToLeftLaneBound(map_pose, bounding_box, lanelet_id); });
  return *std::min_element(distances.begin(), distances.end());
}

auto distanceToRightLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Id lanelet_id)
  -> double
{
  if (const auto bound = lanelet_core::other::getRightBound(lanelet_id); bound.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate right bounds of lanelet_id : ", lanelet_id,
      " please check lanelet map.");
  } else if (const auto polygon =
               math::geometry::transformPoints(map_pose, math::geometry::toPolygon2D(bounding_box));
             polygon.empty()) {
    THROW_SEMANTIC_ERROR("Failed to calculate 2d polygon.");
  } else {
    return math::geometry::getDistance2D(bound, polygon);
  }
}

auto distanceToRightLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids)
  -> double
{
  std::vector<double> distances;
  std::transform(
    lanelet_ids.begin(), lanelet_ids.end(), std::back_inserter(distances),
    [&](auto lanelet_id) { return distanceToLeftLaneBound(map_pose, bounding_box, lanelet_id); });
  return *std::min_element(distances.begin(), distances.end());
}

auto distanceToLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Id lanelet_id)
  -> double
{
  return std::min(
    distanceToLeftLaneBound(map_pose, bounding_box, lanelet_id),
    distanceToRightLaneBound(map_pose, bounding_box, lanelet_id));
}

auto distanceToLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids)
  -> double
{
  return std::min(
    distanceToLeftLaneBound(map_pose, bounding_box, lanelet_ids),
    distanceToRightLaneBound(map_pose, bounding_box, lanelet_ids));
}

auto distanceToTrafficLightStopLine(
  const math::geometry::CatmullRomSplineInterface & spline, const lanelet::Id traffic_light_id)
  -> std::optional<double>
{
  return lanelet_core::distance::getDistanceToTrafficLightStopLine(spline, traffic_light_id);
}

auto distanceToCrosswalk(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints_array,
  const lanelet::Id target_crosswalk_id) -> std::optional<double>
{
  if (waypoints_array.waypoints.empty()) {
    return std::nullopt;
  } else {
    math::geometry::CatmullRomSpline spline(waypoints_array.waypoints);
    return spline.getCollisionPointIn2D(
      lanelet_core::other::getLaneletPolygon(target_crosswalk_id));
  }
}

auto distanceToCrosswalk(
  const math::geometry::CatmullRomSplineInterface & spline, const lanelet::Id target_crosswalk_id)
  -> std::optional<double>
{
  return spline.getCollisionPointIn2D(
    lanelet_core::other::getLaneletPolygon(target_crosswalk_id), false);
}

auto distanceToStopLine(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints_array,
  const lanelet::Id target_stop_line_id) -> std::optional<double>
{
  if (waypoints_array.waypoints.empty()) {
    return std::nullopt;
  } else {
    const math::geometry::CatmullRomSpline spline(waypoints_array.waypoints);
    const auto polygon = lanelet_core::other::getStopLinePolygon(target_stop_line_id);
    return spline.getCollisionPointIn2D(polygon);
  }
}

auto distanceToYieldStop(
  const CanonicalizedLaneletPose & reference_pose, const lanelet::Ids & following_lanelets,
  const std::vector<CanonicalizedLaneletPose> & other_poses) -> std::optional<double>
{
  auto getPosesOnLanelet = [&other_poses](const auto & lanelet_id) {
    std::vector<CanonicalizedLaneletPose> ret;
    for (const auto & pose : other_poses) {
      if (isSameLaneletId(pose, lanelet_id)) {
        ret.emplace_back(pose);
      }
    }
    return ret;
  };

  std::set<double> distances;
  for (const auto & lanelet_id : following_lanelets) {
    const auto right_of_way_ids = lanelet_core::route::getRightOfWayLaneletIds(lanelet_id);
    for (const auto right_of_way_id : right_of_way_ids) {
      const auto other_poses = getPosesOnLanelet(right_of_way_id);
      if (!other_poses.empty()) {
        const auto distance_forward = lanelet_core::distance::getLongitudinalDistance(
          static_cast<LaneletPose>(reference_pose),
          helper::constructLaneletPose(lanelet_id, 0.0, 0.0));
        const auto distance_backward = lanelet_core::distance::getLongitudinalDistance(
          helper::constructLaneletPose(lanelet_id, 0.0, 0.0),
          static_cast<LaneletPose>(reference_pose));
        if (distance_forward) {
          distances.insert(distance_forward.value());
        } else if (distance_backward) {
          distances.insert(-distance_backward.value());
        }
      }
    }
    if (distances.size() != 0) {
      return *distances.begin();
    }
  }
  return std::nullopt;
}

auto splineDistanceToBoundingBox(
  const math::geometry::CatmullRomSplineInterface & spline, const CanonicalizedLaneletPose & pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const double width_extension_right,
  const double width_extension_left, const double length_extension_front,
  const double length_extension_rear) -> std::optional<double>
{
  const auto polygon = math::geometry::transformPoints(
    static_cast<geometry_msgs::msg::Pose>(pose),
    math::geometry::getPointsFromBbox(
      bounding_box, width_extension_right, width_extension_left, length_extension_front,
      length_extension_rear));
  return spline.getCollisionPointIn2D(polygon, false);
}

auto distanceToNearestConflictingPose(
  const lanelet::Ids & following_lanelets, const math::geometry::CatmullRomSplineInterface & spline,
  const std::vector<CanonicalizedEntityStatus> & other_statuses) -> std::optional<double>
{
  const auto conflicting_entities_on_crosswalk =
    [&other_statuses](
      const lanelet::Ids & following_lanelets) -> std::vector<CanonicalizedEntityStatus> {
    std::vector<CanonicalizedEntityStatus> conflicting_entity_status;
    const auto conflicting_crosswalks =
      lanelet_core::route::getConflictingCrosswalkIds(following_lanelets);
    for (const auto & status : other_statuses) {
      if (
        status.laneMatchingSucceed() &&
        std::count(
          conflicting_crosswalks.begin(), conflicting_crosswalks.end(), status.getLaneletId()) >=
          1) {
        conflicting_entity_status.emplace_back(status);
      }
    }
    return conflicting_entity_status;
  }(following_lanelets);

  const auto conflicting_entities_on_lane =
    [&other_statuses](
      const lanelet::Ids & following_lanelets) -> std::vector<CanonicalizedEntityStatus> {
    std::vector<CanonicalizedEntityStatus> conflicting_entity_status;
    const auto conflicting_lanes = lanelet_core::route::getConflictingLaneIds(following_lanelets);
    for (const auto & status : other_statuses) {
      if (
        status.laneMatchingSucceed() &&
        std::count(conflicting_lanes.begin(), conflicting_lanes.end(), status.getLaneletId()) >=
          1) {
        conflicting_entity_status.emplace_back(status);
      }
    }
    return conflicting_entity_status;
  }(following_lanelets);

  std::set<double> distances;
  for (const auto & status : conflicting_entities_on_crosswalk) {
    if (const auto pose = status.getCanonicalizedLaneletPose()) {
      if (const auto s = distanceToCrosswalk(spline, pose->getLaneletId())) {
        distances.insert(s.value());
      }
    }
  }
  for (const auto & status : conflicting_entities_on_lane) {
    if (const auto pose = status.getCanonicalizedLaneletPose()) {
      if (
        const auto s = splineDistanceToBoundingBox(
          spline, pose.value(), status.getBoundingBox(), 0.0, 0.0, 0.0, 1.0)) {
        distances.insert(s.value());
      }
    }
  }

  if (distances.empty()) {
    return std::nullopt;
  } else {
    return *distances.begin();
  }
}
}  // namespace distance
}  // namespace traffic_simulator
