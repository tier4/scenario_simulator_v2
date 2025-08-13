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
#include <geometry/distance.hpp>
#include <geometry/transform.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/distance.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>

namespace traffic_simulator
{
inline namespace distance
{
auto lateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  const RoutingConfiguration & routing_configuration) -> std::optional<double>
{
  return lanelet_wrapper::distance::lateralDistance(
    static_cast<LaneletPose>(from), static_cast<LaneletPose>(to), routing_configuration);
}

auto lateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  double matching_distance, const RoutingConfiguration & routing_configuration)
  -> std::optional<double>
{
  if (
    std::abs(static_cast<LaneletPose>(from).offset) <= matching_distance &&
    std::abs(static_cast<LaneletPose>(to).offset) <= matching_distance) {
    return lateralDistance(from, to, routing_configuration);
  } else {
    return std::nullopt;
  }
}

auto countLaneChanges(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  const traffic_simulator::RoutingConfiguration & routing_configuration,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<std::pair<int, int>>
{
  return hdmap_utils_ptr->countLaneChanges(
    static_cast<LaneletPose>(from), static_cast<LaneletPose>(to), routing_configuration);
}

/// @sa https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/docs/developer_guide/lane_pose_calculation/GetLongitudinalDistance.md
auto longitudinalDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool const include_adjacent_lanelet, bool const include_opposite_direction,
  const RoutingConfiguration & routing_configuration) -> std::optional<double>
{
  if (!include_adjacent_lanelet) {
    auto to_canonicalized = static_cast<LaneletPose>(to);
    if (to.hasAlternativeLaneletPose()) {
      if (
        const auto to_canonicalized_opt = to.getAlternativeLaneletPoseBaseOnShortestRouteFrom(
          static_cast<LaneletPose>(from), routing_configuration)) {
        to_canonicalized = to_canonicalized_opt.value();
      }
    }

    const auto forward_distance = lanelet_wrapper::distance::longitudinalDistance(
      static_cast<LaneletPose>(from), to_canonicalized, routing_configuration);

    const auto backward_distance = lanelet_wrapper::distance::longitudinalDistance(
      to_canonicalized, static_cast<LaneletPose>(from), routing_configuration);

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
    /**
     * @brief A matching distance of about 1.5*lane widths is given as the matching distance to match the
     * Entity present on the adjacent Lanelet.
     * The length of the horizontal bar must intersect with the adjacent lanelet,
     * so it is always 10m regardless of the entity type.
     */
    constexpr double matching_distance = 5.0;

    auto from_poses = lanelet_wrapper::pose::toLaneletPoses(
      static_cast<geometry_msgs::msg::Pose>(from), static_cast<LaneletPose>(from).lanelet_id,
      matching_distance, include_opposite_direction, routing_configuration.routing_graph_type);
    from_poses.emplace_back(from);

    auto to_poses = lanelet_wrapper::pose::toLaneletPoses(
      static_cast<geometry_msgs::msg::Pose>(to), static_cast<LaneletPose>(to).lanelet_id,
      matching_distance, include_opposite_direction, routing_configuration.routing_graph_type);
    to_poses.emplace_back(to);

    std::vector<double> distances = {};
    for (const auto & from_pose : from_poses) {
      for (const auto & to_pose : to_poses) {
        if (
          const auto distance = distance::longitudinalDistance(
            CanonicalizedLaneletPose(from_pose), CanonicalizedLaneletPose(to_pose), false,
            include_opposite_direction, routing_configuration)) {
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
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box,
  const RoutingConfiguration & routing_configuration) -> std::optional<double>
{
  if (const auto lateral_distance = lateralDistance(from, to, routing_configuration);
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
  const RoutingConfiguration & routing_configuration) -> std::optional<double>
{
  if (const auto longitudinal_distance = distance::longitudinalDistance(
        from, to, include_adjacent_lanelet, include_opposite_direction, routing_configuration);
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

auto boundingBoxLaneLongitudinalDistance(
  const std::optional<double> & longitudinal_distance,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box) -> std::optional<double>
{
  if (longitudinal_distance) {
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

// Bounds
auto distanceToLeftLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Id lanelet_id)
  -> double
{
  if (const auto bound = lanelet_wrapper::lanelet_map::leftBound(lanelet_id); bound.empty()) {
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
  if (lanelet_ids.empty()) {
    THROW_SEMANTIC_ERROR("Failing to calculate distanceToLeftLaneBound given an empty vector.");
  }
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & lanelet_id : lanelet_ids) {
    const auto distance = distanceToLeftLaneBound(map_pose, bounding_box, lanelet_id);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

auto distanceToRightLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Id lanelet_id)
  -> double
{
  if (const auto & bound = lanelet_wrapper::lanelet_map::rightBound(lanelet_id); bound.empty()) {
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
  if (lanelet_ids.empty()) {
    THROW_SEMANTIC_ERROR("Failing to calculate distanceToRightLaneBound for given empty vector.");
  }
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & lanelet_id : lanelet_ids) {
    const double distance = distanceToRightLaneBound(map_pose, bounding_box, lanelet_id);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
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

auto distanceToCrosswalk(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints_array,
  const lanelet::Id target_crosswalk_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>
{
  if (waypoints_array.waypoints.empty()) {
    return std::nullopt;
  } else {
    math::geometry::CatmullRomSpline spline(waypoints_array.waypoints);
    auto polygon = hdmap_utils_ptr->getLaneletPolygon(target_crosswalk_id);
    return spline.getCollisionPointIn2D(polygon);
  }
}

auto distanceAlongLanelet(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const geometry_msgs::msg::Pose & to_pose,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box, const double matching_distance,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>
{
  /// @note due to this hardcoded value, the method cannot be used for calculations along a crosswalk (for pedestrians)
  constexpr bool include_crosswalk = false;
  constexpr bool include_adjacent_lanelet = false;
  constexpr bool include_opposite_direction = false;
  constexpr bool allow_lane_change = true;

  const auto routing_configuration = traffic_simulator::RoutingConfiguration{allow_lane_change};

  if (const auto from_canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
        from_pose, from_bounding_box, include_crosswalk, matching_distance);
      from_canonicalized_lanelet_pose.has_value()) {
    if (const auto to_canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
          to_pose, to_bounding_box, include_crosswalk, matching_distance);
        to_canonicalized_lanelet_pose.has_value()) {
      return distance::longitudinalDistance(
        from_canonicalized_lanelet_pose.value(), to_canonicalized_lanelet_pose.value(),
        include_adjacent_lanelet, include_opposite_direction, routing_configuration);
      longitudinal_distance.has_value()
    }
  }
  return std::nullopt;
}

auto distanceToSpline(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box,
  const math::geometry::CatmullRomSplineInterface & spline, const double s_reference) -> double
{
  /*
  * Convergence threshold for binary search.
  * The search stops when the interval between `s_start` and `s_end` is below this value.
  * The value 0.05 was chosen empirically to balance accuracy and performance.
  * A smaller value improves precision but increases computation time.
  */
  constexpr double distance_accuracy{0.05};

  const auto bounding_box_map_points =
    math::geometry::transformPoints(map_pose, math::geometry::getPointsFromBbox(bounding_box));
  const auto bounding_box_diagonal_length =
    math::geometry::getDistance(bounding_box_map_points[0], bounding_box_map_points[2]);

  /// @note it may be a good idea to develop spline.getSquaredDistanceIn2D(point, s_start, s_end);
  std::vector<double> distances;
  for (const auto & point : bounding_box_map_points) {
    auto s_start = s_reference - bounding_box_diagonal_length / 2;
    auto s_end = s_reference + bounding_box_diagonal_length / 2;
    auto s_start_distance = spline.getSquaredDistanceIn2D(point, s_start);
    auto s_end_distance = spline.getSquaredDistanceIn2D(point, s_end);

    while (std::abs(s_start - s_end) > distance_accuracy) {
      double s_mid = s_start + (s_end - s_start) / 2;
      double s_mid_distance = spline.getSquaredDistanceIn2D(point, s_mid);
      if (s_start_distance > s_end_distance) {
        s_start = s_mid;
        s_start_distance = s_mid_distance;
      } else {
        s_end = s_mid;
        s_end_distance = s_mid_distance;
      }
    }
    distances.push_back(std::min(s_start_distance, s_end_distance));
  }
  return std::sqrt(*std::min_element(distances.begin(), distances.end()));
}
}  // namespace distance
}  // namespace traffic_simulator
