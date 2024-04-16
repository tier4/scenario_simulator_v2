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

#include <traffic_simulator/distance_utils.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>

namespace traffic_simulator
{
auto DistanceUtils::canonicalize(
  const LaneletPose & lanelet_pose,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> CanonicalizedLaneletPose
{
  return CanonicalizedLaneletPose(lanelet_pose, hdmap_utils_ptr);
}

auto DistanceUtils::toMapPose(const CanonicalizedLaneletPose & lanelet_pose)
  -> const geometry_msgs::msg::Pose
{
  return static_cast<geometry_msgs::msg::Pose>(lanelet_pose);
}

auto DistanceUtils::toLaneletPose(
  const geometry_msgs::msg::Pose & map_pose, bool include_crosswalk,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<CanonicalizedLaneletPose>
{
  if (const auto pose = hdmap_utils_ptr->toLaneletPose(map_pose, include_crosswalk)) {
    return DistanceUtils::canonicalize(pose.value(), hdmap_utils_ptr);
  }
  return std::nullopt;
}

auto DistanceUtils::getLateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  double matching_distance, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>
{
  if (
    std::abs(static_cast<LaneletPose>(from).offset) <= matching_distance &&
    std::abs(static_cast<LaneletPose>(to).offset) <= matching_distance) {
    return getLateralDistance(from, to, allow_lane_change, hdmap_utils_ptr);
  }
  return std::nullopt;
}

auto DistanceUtils::getLateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<double>
{
  return hdmap_utils_ptr->getLateralDistance(
    static_cast<LaneletPose>(from), static_cast<LaneletPose>(to), allow_lane_change);
}

auto DistanceUtils::getLongitudinalDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool include_adjacent_lanelet, bool include_opposite_direction, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>
{
  if (!include_adjacent_lanelet) {
    auto to_canonicalized = static_cast<LaneletPose>(to);
    if (to.hasAlternativeLaneletPose()) {
      if (
        const auto to_canonicalized_optional = to.getAlternativeLaneletPoseBaseOnShortestRouteFrom(
          static_cast<LaneletPose>(from), hdmap_utils_ptr, allow_lane_change)) {
        to_canonicalized = to_canonicalized_optional.value();
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
     * @brief hard coded parameter!! 5.0 is a matching distance of the toLaneletPoses function.
     * A matching distance of about 1.5 lane widths is given as the matching distance to match the
     * Entity present on the adjacent Lanelet.
     */
    auto from_poses = hdmap_utils_ptr->toLaneletPoses(
      static_cast<geometry_msgs::msg::Pose>(from), static_cast<LaneletPose>(from).lanelet_id, 5.0,
      include_opposite_direction);
    from_poses.emplace_back(from);
    /**
     * @brief hard coded parameter!! 5.0 is a matching distance of the toLaneletPoses function.
     * A matching distance of about 1.5 lane widths is given as the matching distance to match the
     * Entity present on the adjacent Lanelet.
     */
    auto to_poses = hdmap_utils_ptr->toLaneletPoses(
      static_cast<geometry_msgs::msg::Pose>(to), static_cast<LaneletPose>(to).lanelet_id, 5.0,
      include_opposite_direction);
    to_poses.emplace_back(to);
    std::vector<double> distances = {};
    for (const auto & from_pose : from_poses) {
      for (const auto & to_pose : to_poses) {
        if (
          const auto distance = getLongitudinalDistance(
            CanonicalizedLaneletPose(from_pose, hdmap_utils_ptr),
            CanonicalizedLaneletPose(to_pose, hdmap_utils_ptr), false, include_opposite_direction,
            allow_lane_change, hdmap_utils_ptr)) {
          distances.emplace_back(distance.value());
        }
      }
    }
    if (distances.empty()) {
      return std::nullopt;
    }
    std::sort(distances.begin(), distances.end(), [](double a, double b) {
      return std::abs(a) < std::abs(b);
    });
    return distances.front();
  }
}

auto DistanceUtils::makeNativeRelativeLanePosition(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  const auto longitudinal_distance = DistanceUtils ::getLongitudinalDistance(
    from, to, include_adjacent_lanelet, include_opposite_direction, allow_lane_change,
    hdmap_utils_ptr);
  const auto lateral_distance =
    DistanceUtils::getLateralDistance(from, to, allow_lane_change, hdmap_utils_ptr);

  traffic_simulator::LaneletPose position =
    traffic_simulator::lanelet_pose::createQuietNaNLaneletPose();
  if (longitudinal_distance && lateral_distance) {
    position.s = longitudinal_distance.value();
    position.offset = lateral_distance.value();
  }
  return position;
}

// auto DistanceUtils::getBoundingBoxDistance(const std::string & from, const std::string & to)
//   -> std::optional<double>
// {
//   return math::geometry::getPolygonDistance(
//     getMapPose(from), getBoundingBox(from), getMapPose(to), getBoundingBox(to));
// }

auto DistanceUtils::getBoundingBoxDistance(
  const geometry_msgs::msg::Pose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const geometry_msgs::msg::Pose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox)
  -> std::optional<double>
{
  return math::geometry::getPolygonDistance(from, from_bbox, to, to_bbox);
}

auto DistanceUtils::getBoundingBoxLaneLateralDistance(
  const CanonicalizedLaneletPose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const CanonicalizedLaneletPose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> std::optional<double>
{
  if (const auto lateral_distance =
        getLateralDistance(from, to, allow_lane_change, hdmap_utils_ptr);
      lateral_distance) {
    const auto from_bbox_distances = math::geometry::getDistancesFromCenterToEdge(from_bbox);
    const auto to_bbox_distances = math::geometry::getDistancesFromCenterToEdge(to_bbox);
    auto bbox_distance = 0.0;

    if (lateral_distance.value() > 0) {
      bbox_distance = -std::abs(from_bbox_distances.right) - std::abs(to_bbox_distances.left);
    } else if (lateral_distance.value() < 0) {
      bbox_distance = std::abs(from_bbox_distances.left) + std::abs(to_bbox_distances.right);
    }
    return lateral_distance.value() + bbox_distance;
  }
  return std::nullopt;
}

// auto DistanceUtils::getBoundingBoxLaneLateralDistance(
//   const std::string & from, const CanonicalizedLaneletPose & to, bool allow_lane_change)
//   -> std::optional<double>
// {
//   if (const auto from_pose = getLaneletPose(from); from_pose) {
//     traffic_simulator_msgs::msg::BoundingBox bbox_empty;
//     return getBoundingBoxLaneLateralDistance(
//       from_pose.value(), getBoundingBox(from), to, bbox_empty, allow_lane_change);
//   }
//   return std::nullopt;
// }

// auto DistanceUtils::getBoundingBoxLaneLateralDistance(
//   const std::string & from, const std::string & to, bool allow_lane_change) ->
//   std::optional<double>
// {
//   const auto from_pose = getLaneletPose(from);
//   const auto to_pose = getLaneletPose(to);
//   if (from_pose and to_pose) {
//     return getBoundingBoxLaneLateralDistance(
//       from_pose.value(), getBoundingBox(from), to_pose.value(), getBoundingBox(to),
//       allow_lane_change);
//   }
//   return std::nullopt;
// }

auto DistanceUtils::getBoundingBoxLaneLongitudinalDistance(
  const CanonicalizedLaneletPose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const CanonicalizedLaneletPose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox,
  bool include_adjacent_lanelet, bool include_opposite_direction, bool allow_lane_change,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> std::optional<double>
{
  if (const auto longitudinal_distance = getLongitudinalDistance(
        from, to, include_adjacent_lanelet, include_opposite_direction, allow_lane_change,
        hdmap_utils_ptr);
      longitudinal_distance) {
    const auto from_bbox_distances = math::geometry::getDistancesFromCenterToEdge(from_bbox);
    const auto to_bbox_distances = math::geometry::getDistancesFromCenterToEdge(to_bbox);
    auto bbox_distance = 0.0;

    if (longitudinal_distance.value() > 0.0) {
      bbox_distance = -std::abs(from_bbox_distances.front) - std::abs(to_bbox_distances.rear);
    } else if (longitudinal_distance.value() < 0.0) {
      bbox_distance = +std::abs(from_bbox_distances.rear) + std::abs(to_bbox_distances.front);
    }
    return longitudinal_distance.value() + bbox_distance;
  }
  return std::nullopt;
}

// auto DistanceUtils::getBoundingBoxLaneLongitudinalDistance(
//   const std::string & from, const CanonicalizedLaneletPose & to, bool include_adjacent_lanelet,
//   bool include_opposite_direction, bool allow_lane_change) -> std::optional<double>
// {
//   const auto from_pose = getLaneletPose(from);
//   if (laneMatchingSucceed(from) and from_pose) {
//     traffic_simulator_msgs::msg::BoundingBox bbox_empty;
//     return getBoundingBoxLaneLongitudinalDistance(
//       from_pose.value(), getBoundingBox(from), to, bbox_empty, include_adjacent_lanelet,
//       include_opposite_direction, allow_lane_change);
//   }
//   return std::nullopt;
// }
// auto DistanceUtils::getBoundingBoxLaneLongitudinalDistance(
//   const std::string & from, const std::string & to, bool include_adjacent_lanelet,
//   bool include_opposite_direction, bool allow_lane_change) -> std::optional<double>
// {
//   const auto from_pose = getLaneletPose(from);
//   const auto to_pose = getLaneletPose(to);

//   if (laneMatchingSucceed(from) and from_pose and laneMatchingSucceed(to) and to_pose) {
//     return getBoundingBoxLaneLongitudinalDistance(
//       from_pose.value(), getBoundingBox(from), to_pose.value(), getBoundingBox(to),
//       include_adjacent_lanelet, include_opposite_direction, allow_lane_change);
//   }
//   return std::nullopt;
// }

auto DistanceUtils::makeNativeBoundingBoxRelativeLanePosition(
  const CanonicalizedLaneletPose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const CanonicalizedLaneletPose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox,
  bool allow_lane_change, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  -> traffic_simulator::LaneletPose
{
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};

  const auto longitudinal_bb_distance = DistanceUtils ::getBoundingBoxLaneLongitudinalDistance(
    from, from_bbox, to, to_bbox, include_adjacent_lanelet, include_opposite_direction,
    allow_lane_change, hdmap_utils_ptr);
  const auto lateral_bb_distance = DistanceUtils::getBoundingBoxLaneLateralDistance(
    from, from_bbox, to, to_bbox, allow_lane_change, hdmap_utils_ptr);

  traffic_simulator::LaneletPose position =
    traffic_simulator::lanelet_pose::createQuietNaNLaneletPose();
  if (longitudinal_bb_distance && include_opposite_direction) {
    position.s = longitudinal_bb_distance.value();
    position.offset = lateral_bb_distance.value();
  }
  return position;
}

auto DistanceUtils::getBoundingBoxRelativePose(
  const geometry_msgs::msg::Pose & from, const traffic_simulator_msgs::msg::BoundingBox & from_bbox,
  const geometry_msgs::msg::Pose & to, const traffic_simulator_msgs::msg::BoundingBox & to_bbox)
  -> std::optional<geometry_msgs::msg::Pose>
{
  if (const auto closest_points = math::geometry::getClosestPoses(from, from_bbox, to, to_bbox);
      closest_points) {
    const auto from_pose_bbox = getRelativePose(from, closest_points.value().first);
    const auto to_pose_bbox = getRelativePose(from, closest_points.value().second);
    if (from_pose_bbox && to_pose_bbox) {
      return math::geometry::subtractPoses(from_pose_bbox.value(), to_pose_bbox.value());
    }
  }
  return std::nullopt;
}

// auto DistanceUtils::getBoundingBoxRelativePose(
//   const std::string & from, const geometry_msgs::msg::Pose & to)
//   -> std::optional<geometry_msgs::msg::Pose>
// {
//   traffic_simulator_msgs::msg::BoundingBox bbox_empty;
//   return getBoundingBoxRelativePose(getMapPose(from), getBoundingBox(from), to, bbox_empty);
// }

// auto DistanceUtils::getBoundingBoxRelativePose(const std::string & from, const std::string & to)
//   -> std::optional<geometry_msgs::msg::Pose>
// {
//   return getBoundingBoxRelativePose(
//     getMapPose(from), getBoundingBox(from), getMapPose(to), getBoundingBox(to));
// }

// auto DistanceUtils::getDistanceToLaneBound() -> double
// {
//   return std::min(getDistanceToLeftLaneBound(), getDistanceToRightLaneBound());
// }

auto DistanceUtils::getDistanceToLaneBound(
  const CanonicalizedEntityStatus & status, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  return std::min(
    getDistanceToLeftLaneBound(status, lanelet_id, hdmap_utils_ptr),
    getDistanceToRightLaneBound(status, lanelet_id, hdmap_utils_ptr));
}

auto DistanceUtils::getDistanceToLaneBound(
  const CanonicalizedEntityStatus & status, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  return std::min(
    getDistanceToLeftLaneBound(status, lanelet_ids, hdmap_utils_ptr),
    getDistanceToRightLaneBound(status, lanelet_ids, hdmap_utils_ptr));
}

// auto DistanceUtils::getDistanceToLeftLaneBound() -> double
// {
//   return getDistanceToLeftLaneBound(getRouteLanelets());
// }

auto DistanceUtils::getDistanceToLeftLaneBound(
  const CanonicalizedEntityStatus & status, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  std::vector<double> distances;
  std::transform(
    lanelet_ids.begin(), lanelet_ids.end(), std::back_inserter(distances), [&](auto lanelet_id) {
      return getDistanceToLeftLaneBound(status, lanelet_id, hdmap_utils_ptr);
    });
  return *std::min_element(distances.begin(), distances.end());
}

auto DistanceUtils::getDistanceToLeftLaneBound(
  const CanonicalizedEntityStatus & status, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  if (const auto bound = hdmap_utils_ptr->getLeftBound(lanelet_id); bound.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate left bounds of lanelet_id : ", lanelet_id, " please check lanelet map.");
  } else if (const auto polygon = math::geometry::transformPoints(
               status.getMapPose(), get2DPolygon(status.getBoundingBox()));
             polygon.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate 2d polygon of entity: ", status.getName(), " . Please check ",
      status.getName(), " exists and it's definition");
  } else {
    return math::geometry::getDistance2D(bound, polygon);
  }
}

// auto DistanceUtils::getDistanceToRightLaneBound() -> double
// {
//   return getDistanceToRightLaneBound(getRouteLanelets());
// }

auto DistanceUtils::getDistanceToRightLaneBound(
  const CanonicalizedEntityStatus & status, const lanelet::Ids & lanelet_ids,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  std::vector<double> distances;
  std::transform(
    lanelet_ids.begin(), lanelet_ids.end(), std::back_inserter(distances), [&](auto lanelet_id) {
      return getDistanceToLeftLaneBound(status, lanelet_id, hdmap_utils_ptr);
    });
  return *std::min_element(distances.begin(), distances.end());
}

auto DistanceUtils::getDistanceToRightLaneBound(
  const CanonicalizedEntityStatus & status, lanelet::Id lanelet_id,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) -> double
{
  if (const auto bound = hdmap_utils_ptr->getRightBound(lanelet_id); bound.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate right bounds of lanelet_id : ", lanelet_id,
      " please check lanelet map.");
  } else if (const auto polygon = math::geometry::transformPoints(
               status.getMapPose(), get2DPolygon(status.getBoundingBox()));
             polygon.empty()) {
    THROW_SEMANTIC_ERROR(
      "Failed to calculate 2d polygon of entity: ", status.getName(), " . Please check ",
      status.getName(), " exists and it's definition");
  } else {
    return math::geometry::getDistance2D(bound, polygon);
  }
}

auto DistanceUtils::getDistanceToCrosswalk(
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

auto DistanceUtils::getDistanceToStopLine(
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

auto DistanceUtils::getRelativePose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  try {
    return math::geometry::getRelativePose(from, to);
  } catch (...) {
    return std::nullopt;
  }
}

// auto EntityManager::getRelativePose(
//   const geometry_msgs::msg::Pose & from, const std::string & to) ->
//   geometry_msgs::msg::Pose
// {
//   return getRelativePose(from, getMapPose(to));
// }

// auto DistanceUtils::getRelativePose(
//   const std::string & from, const geometry_msgs::msg::Pose & to) ->
//   geometry_msgs::msg::Pose
// {
//   return getRelativePose(getMapPose(from), to);
// }

// auto EntityManager::getRelativePose(const std::string & from, const std::string & to)
//   -> geometry_msgs::msg::Pose
// {
//   return getRelativePose(getMapPose(from), getMapPose(to));
// }

auto DistanceUtils::getRelativePose(
  const geometry_msgs::msg::Pose & from, const CanonicalizedLaneletPose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  return getRelativePose(from, static_cast<geometry_msgs::msg::Pose>(to));
}

auto DistanceUtils::getRelativePose(
  const CanonicalizedLaneletPose & from, const geometry_msgs::msg::Pose & to)
  -> std::optional<geometry_msgs::msg::Pose>
{
  return getRelativePose(static_cast<geometry_msgs::msg::Pose>(from), to);
}

// auto EntityManager::getRelativePose(
//   const std::string & from, const CanonicalizedLaneletPose & to) ->
//   geometry_msgs::msg::Pose
// {
//   return getRelativePose(getMapPose(from), toMapPose(to));
// }

// auto EntityManager::getRelativePose(
//   const CanonicalizedLaneletPose & from, const std::string & to) ->
//   geometry_msgs::msg::Pose
// {
//   return getRelativePose(toMapPose(from), getMapPose(to));
// }

auto DistanceUtils::get2DPolygon(const traffic_simulator_msgs::msg::BoundingBox & bounding_box)
  -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> points_bbox;
  geometry_msgs::msg::Point p0, p1, p2, p3, p4, p5, p6, p7;

  p0.x = bounding_box.center.x + bounding_box.dimensions.x * 0.5;
  p0.y = bounding_box.center.y + bounding_box.dimensions.y * 0.5;
  p0.z = bounding_box.center.z + bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p0);

  p1.x = bounding_box.center.x + bounding_box.dimensions.x * 0.5;
  p1.y = bounding_box.center.y + bounding_box.dimensions.y * 0.5;
  p1.z = bounding_box.center.z - bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p1);

  p2.x = bounding_box.center.x + bounding_box.dimensions.x * 0.5;
  p2.y = bounding_box.center.y - bounding_box.dimensions.y * 0.5;
  p2.z = bounding_box.center.z + bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p2);

  p3.x = bounding_box.center.x - bounding_box.dimensions.x * 0.5;
  p3.y = bounding_box.center.y + bounding_box.dimensions.y * 0.5;
  p3.z = bounding_box.center.z + bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p3);

  p4.x = bounding_box.center.x + bounding_box.dimensions.x * 0.5;
  p4.y = bounding_box.center.y - bounding_box.dimensions.y * 0.5;
  p4.z = bounding_box.center.z - bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p4);

  p5.x = bounding_box.center.x - bounding_box.dimensions.x * 0.5;
  p5.y = bounding_box.center.y + bounding_box.dimensions.y * 0.5;
  p5.z = bounding_box.center.z - bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p5);

  p6.x = bounding_box.center.x - bounding_box.dimensions.x * 0.5;
  p6.y = bounding_box.center.y - bounding_box.dimensions.y * 0.5;
  p6.z = bounding_box.center.z + bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p6);

  p7.x = bounding_box.center.x - bounding_box.dimensions.x * 0.5;
  p7.y = bounding_box.center.y - bounding_box.dimensions.y * 0.5;
  p7.z = bounding_box.center.z - bounding_box.dimensions.z * 0.5;
  points_bbox.emplace_back(p7);

  return math::geometry::get2DConvexHull(points_bbox);
}
}  // namespace traffic_simulator
