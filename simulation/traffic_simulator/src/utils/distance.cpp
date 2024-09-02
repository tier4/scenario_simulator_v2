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
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>

namespace traffic_simulator
{
namespace distance
{
auto lateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<double>
{
  return hdmap_utils_ptr->getLateralDistance(
    static_cast<LaneletPose>(from), static_cast<LaneletPose>(to), allow_lane_change);
}

auto lateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  double matching_distance, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>
{
  if (
    std::abs(static_cast<LaneletPose>(from).offset) <= matching_distance &&
    std::abs(static_cast<LaneletPose>(to).offset) <= matching_distance) {
    return lateralDistance(from, to, allow_lane_change, hdmap_utils_ptr);
  } else {
    return std::nullopt;
  }
}

auto countLaneChanges(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<std::pair<int, int>>
{
  return hdmap_utils_ptr->countLaneChanges(
    static_cast<LaneletPose>(from), static_cast<LaneletPose>(to), allow_lane_change);
}

/// @sa https://github.com/tier4/scenario_simulator_v2/blob/729e4e6372cdba60e377ae097d032905b80763a9/docs/developer_guide/lane_pose_calculation/GetLongitudinalDistance.md
auto longitudinalDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool include_adjacent_lanelet, bool include_opposite_direction, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>
{
  if (!include_adjacent_lanelet) {
    auto to_canonicalized = static_cast<LaneletPose>(to);
    if (to.hasAlternativeLaneletPose()) {
      if (
        const auto to_canonicalized_opt = to.getAlternativeLaneletPoseBaseOnShortestRouteFrom(
          static_cast<LaneletPose>(from), hdmap_utils_ptr, allow_lane_change)) {
        to_canonicalized = to_canonicalized_opt.value();
      }
    }

    const auto forward_distance = hdmap_utils_ptr->getLongitudinalDistance(
      static_cast<LaneletPose>(from), to_canonicalized, allow_lane_change);

    const auto backward_distance = hdmap_utils_ptr->getLongitudinalDistance(
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
    /**
     * @brief A matching distance of about 1.5*lane widths is given as the matching distance to match the
     * Entity present on the adjacent Lanelet.
     * The length of the horizontal bar must intersect with the adjacent lanelet, 
     * so it is always 10m regardless of the entity type.
     */
    constexpr double matching_distance = 5.0;

    auto from_poses = hdmap_utils_ptr->toLaneletPoses(
      static_cast<geometry_msgs::msg::Pose>(from), static_cast<LaneletPose>(from).lanelet_id,
      matching_distance, include_opposite_direction);
    from_poses.emplace_back(from);

    auto to_poses = hdmap_utils_ptr->toLaneletPoses(
      static_cast<geometry_msgs::msg::Pose>(to), static_cast<LaneletPose>(to).lanelet_id,
      matching_distance, include_opposite_direction);
    to_poses.emplace_back(to);

    std::vector<double> distances = {};
    for (const auto & from_pose : from_poses) {
      for (const auto & to_pose : to_poses) {
        if (
          const auto distance = longitudinalDistance(
            CanonicalizedLaneletPose(from_pose, hdmap_utils_ptr),
            CanonicalizedLaneletPose(to_pose, hdmap_utils_ptr), false, include_opposite_direction,
            allow_lane_change, hdmap_utils_ptr)) {
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
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>
{
  if (const auto lateral_distance = lateralDistance(from, to, allow_lane_change, hdmap_utils_ptr);
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
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box, bool include_adjacent_lanelet,
  bool include_opposite_direction, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>
{
  if (const auto longitudinal_distance = longitudinalDistance(
        from, to, include_adjacent_lanelet, include_opposite_direction, allow_lane_change,
        hdmap_utils_ptr);
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
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  if (const auto bound = hdmap_utils_ptr->getLeftBound(lanelet_id); bound.empty()) {
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
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  std::vector<double> distances;
  std::transform(
    lanelet_ids.begin(), lanelet_ids.end(), std::back_inserter(distances), [&](auto lanelet_id) {
      return distanceToLeftLaneBound(map_pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    });
  return *std::min_element(distances.begin(), distances.end());
}

auto distanceToRightLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  if (const auto bound = hdmap_utils_ptr->getRightBound(lanelet_id); bound.empty()) {
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
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  std::vector<double> distances;
  std::transform(
    lanelet_ids.begin(), lanelet_ids.end(), std::back_inserter(distances), [&](auto lanelet_id) {
      return distanceToLeftLaneBound(map_pose, bounding_box, lanelet_id, hdmap_utils_ptr);
    });
  return *std::min_element(distances.begin(), distances.end());
}

auto distanceToLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  return std::min(
    distanceToLeftLaneBound(map_pose, bounding_box, lanelet_id, hdmap_utils_ptr),
    distanceToRightLaneBound(map_pose, bounding_box, lanelet_id, hdmap_utils_ptr));
}

auto distanceToLaneBound(
  const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  return std::min(
    distanceToLeftLaneBound(map_pose, bounding_box, lanelet_ids, hdmap_utils_ptr),
    distanceToRightLaneBound(map_pose, bounding_box, lanelet_ids, hdmap_utils_ptr));
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

auto distanceToStopLine(
  const traffic_simulator_msgs::msg::WaypointsArray & waypoints_array,
  const lanelet::Id target_stop_line_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>
{
  if (waypoints_array.waypoints.empty()) {
    return std::nullopt;
  } else {
    const math::geometry::CatmullRomSpline spline(waypoints_array.waypoints);
    const auto polygon = hdmap_utils_ptr->getStopLinePolygon(target_stop_line_id);
    return spline.getCollisionPointIn2D(polygon);
  }
}
}  // namespace distance
}  // namespace traffic_simulator
