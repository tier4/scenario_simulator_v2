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
#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/lanelet_wrapper/route.hpp>
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

    std::optional<double> min_distance = std::nullopt;
    for (const auto & from_pose : from_poses) {
      for (const auto & to_pose : to_poses) {
        if (const auto distance = distance::longitudinalDistance(
              CanonicalizedLaneletPose(from_pose), CanonicalizedLaneletPose(to_pose), false,
              include_opposite_direction, routing_configuration);
            distance.has_value() and
            (not min_distance.has_value() or
             (std::abs(distance.value()) < std::abs(min_distance.value())))) {
          min_distance = distance;
        }
      }
    }
    return min_distance;
  }
}

// BoundingBox
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

auto splineDistanceToBoundingBox(
  const math::geometry::CatmullRomSplineInterface & spline,
  const CanonicalizedLaneletPose & from_lanelet_pose,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const CanonicalizedLaneletPose & target_lanelet_pose,
  const traffic_simulator_msgs::msg::BoundingBox & target_bounding_box,
  const double lateral_collision_threshold) -> std::optional<double>
{
  const auto [min_range, max_range] = spline.getAltitudeRange();
  if (not lanelet_wrapper::pose::isAltitudeWithinRange(
        target_lanelet_pose.getAltitude(), min_range, max_range)) {
    return std::nullopt;
  }
  /**
   * boundingBoxLaneLongitudinalDistance requires routing_configuration,
   * allow_lane_change = true is needed to check distances to entities on neighbour lanelets
   */
  traffic_simulator::RoutingConfiguration routing_configuration;
  routing_configuration.allow_lane_change = true;
  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{true};
  constexpr bool search_backward{false};

  if (const auto & target_lanelet_pose_alternative =
        traffic_simulator::pose::findRoutableAlternativeLaneletPoseFrom(
          from_lanelet_pose.getLaneletId(), target_lanelet_pose, target_bounding_box);
      target_lanelet_pose_alternative) {
    const auto bounding_box_map_points = math::geometry::transformPoints(
      static_cast<geometry_msgs::msg::Pose>(target_lanelet_pose_alternative.value()),
      math::geometry::getPointsFromBbox(target_bounding_box));
    const auto bounding_box_diagonal_length =
      math::geometry::getDistance(bounding_box_map_points[0], bounding_box_map_points[2]);
    if (const auto longitudinal_distance = distance::longitudinalDistance(
          from_lanelet_pose, target_lanelet_pose_alternative.value(), include_adjacent_lanelet,
          include_opposite_direction, routing_configuration);
        not longitudinal_distance) {
      return std::nullopt;
    } else if (const auto bounding_box_distance =
                 traffic_simulator::distance::boundingBoxLaneLongitudinalDistance(
                   longitudinal_distance, from_bounding_box, target_bounding_box);
               !bounding_box_distance || bounding_box_distance.value() < 0.0) {
      return std::nullopt;
    } else {
      /// @todo rotation of NPC is not taken into account, same as in boundingBoxLaneLongitudinalDistance
      /// this should be considered to be changed in separate task in the future
      const auto target_bounding_box_distance =
        bounding_box_distance.value() +
        math::geometry::getDistancesFromCenterToEdge(from_bounding_box).front;

      const auto target_to_spline_distance = traffic_simulator::distance::distanceToSpline(
        static_cast<geometry_msgs::msg::Pose>(target_lanelet_pose_alternative.value()),
        target_bounding_box, spline, longitudinal_distance.value());

      const double threshold = lateral_collision_threshold < 0.0
                                 ? from_bounding_box.dimensions.y / 2.0
                                 : lateral_collision_threshold;

      if (target_to_spline_distance <= threshold) {
        return target_bounding_box_distance;
      }
      /// @note if the distance of the target entity to the spline cannot be calculated because a collision occurs
      else if (const auto target_polygon = math::geometry::transformPoints(
                 static_cast<geometry_msgs::msg::Pose>(target_lanelet_pose_alternative.value()),
                 math::geometry::getPointsFromBbox(target_bounding_box));
               spline.getCollisionPointIn2D(
                 target_polygon, search_backward,
                 std::make_pair(
                   bounding_box_distance.value(),
                   target_bounding_box_distance + bounding_box_diagonal_length))) {
        return target_bounding_box_distance;
      }
    }
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

// Other objects
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

  std::optional<double> min_distance = std::nullopt;
  auto try_min_distance = [&min_distance](const double & distance) {
    if (not min_distance.has_value() or distance < min_distance.value()) {
      min_distance = distance;
    }
  };
  for (const auto & lanelet_id : following_lanelets) {
    const auto right_of_way_ids = lanelet_wrapper::lanelet_map::rightOfWayLaneletIds(lanelet_id);
    for (const auto right_of_way_id : right_of_way_ids) {
      const auto other_poses_on_lanelet = getPosesOnLanelet(right_of_way_id);
      if (!other_poses_on_lanelet.empty()) {
        const auto distance_forward = lanelet_wrapper::distance::longitudinalDistance(
          static_cast<LaneletPose>(reference_pose),
          helper::constructLaneletPose(lanelet_id, 0.0, 0.0), RoutingConfiguration());
        const auto distance_backward = lanelet_wrapper::distance::longitudinalDistance(
          helper::constructLaneletPose(lanelet_id, 0.0, 0.0),
          static_cast<LaneletPose>(reference_pose), RoutingConfiguration());

        if (distance_forward) {
          try_min_distance(distance_forward.value());
        } else if (distance_backward) {
          try_min_distance(-distance_backward.value());
        }
      }
    }
    if (min_distance.has_value()) {
      return min_distance.value();
    }
  }
  return std::nullopt;
}

static auto distanceToCrossingTraffic(
  const lanelet::Ids & following_lanelets, const math::geometry::CatmullRomSplineInterface & spline,
  const CanonicalizedEntityStatus & from_status,
  const std::vector<CanonicalizedEntityStatus> & other_statuses) -> std::optional<double>
{
  constexpr auto required_gap = 5.0;
  constexpr auto distance_accuracy = 0.05;

  // The other ways through the intersection from the same road. They overlap this lane, so
  // conflictingLaneIds reports them, but they are not traffic to give way to.
  const auto own_road_lane_ids = [](const lanelet::Id id) {
    auto ids = lanelet::Ids();
    for (const auto previous : lanelet_wrapper::lanelet_map::previousLaneletIds(id)) {
      ids.push_back(previous);
      for (const auto next : lanelet_wrapper::lanelet_map::nextLaneletIds(previous)) {
        ids.push_back(next);
      }
    }
    return ids;
  };

  // The lanes that give way to this one, and so may be ignored by it.
  const auto released_lane_ids = [](const lanelet::Id id) {
    const auto & lanelet = lanelet_wrapper::LaneletWrapper::map()->laneletLayer.get(id);
    auto ids = lanelet::Ids();
    for (const auto & right_of_way : lanelet.regulatoryElementsAs<lanelet::RightOfWay>()) {
      if (const auto & prioritized = right_of_way->rightOfWayLanelets();
          std::find(prioritized.begin(), prioritized.end(), lanelet) != prioritized.end()) {
        for (const auto & yield : right_of_way->yieldLanelets()) {
          ids.push_back(yield.id());
        }
      }
    }
    return ids;
  };

  // Whether the map says anything about which of the lanes this one crosses it may ignore.
  const auto has_right_of_way = [](const lanelet::Id id) {
    const auto & lanelet = lanelet_wrapper::LaneletWrapper::map()->laneletLayer.get(id);
    for (const auto & right_of_way : lanelet.regulatoryElementsAs<lanelet::RightOfWay>()) {
      if (const auto & prioritized = right_of_way->rightOfWayLanelets();
          std::find(prioritized.begin(), prioritized.end(), lanelet) != prioritized.end()) {
        return true;
      }
    }
    return false;
  };

  // Whether a lane goes before every lane it crosses. Going straight on a green light does.
  const auto has_priority_over_conflicts = [](const lanelet::Id id) {
    const auto & lanelet = lanelet_wrapper::LaneletWrapper::map()->laneletLayer.get(id);
    const auto & traffic_lights = lanelet.regulatoryElementsAs<lanelet::TrafficLight>();
    if (lanelet.attributeOr("turn_direction", std::string()) == "straight") {
      for (const auto & traffic_light : traffic_lights) {
        if (traffic_light->stopLine()) {
          return true;
        }
      }
    }
    return false;
  };

  const auto edges = math::geometry::getDistancesFromCenterToEdge(from_status.getBoundingBox());

  // Whether the body, stopped with its front the given distance ahead, enters the given lane.
  // It is wider than the line these distances are measured along, so its corners enter first.
  const auto enters = [&](const lanelet::Id id, const double distance) {
    const auto pose = pose::toMapPose(lanelet_wrapper::pose::alongLaneletPose(
      from_status.getLaneletPose(), following_lanelets, distance - edges.front));
    for (const auto side : {edges.left, edges.right}) {
      const auto corner = math::geometry::transformPoint(
        pose, geometry_msgs::build<geometry_msgs::msg::Point>().x(edges.front).y(side).z(0.0));
      if (lanelet_wrapper::lanelet_map::isInLanelet(id, corner)) {
        return true;
      }
    }
    return false;
  };

  // Where the route meets each lane it has to give way to: the ones the given lane crosses,
  // less the ones it may ignore, less the ones the body is in or the route has left behind: a
  // distance to stop short of them is not one that can be stopped at. Empty when the map says
  // nothing, so a lane without a RightOfWay gives way to no one.
  const auto crossing_lane_ids = [&](const lanelet::Id lanelet_id) {
    auto crossings = std::vector<std::pair<lanelet::Id, double>>();
    if (has_right_of_way(lanelet_id) and not has_priority_over_conflicts(lanelet_id)) {
      const auto released = released_lane_ids(lanelet_id);
      const auto owning = own_road_lane_ids(lanelet_id);
      for (const auto id : lanelet_wrapper::lanelet_map::conflictingLaneIds({lanelet_id})) {
        if (
          std::find(released.begin(), released.end(), id) == released.end() and
          std::find(owning.begin(), owning.end(), id) == owning.end() and
          not enters(id, edges.front)) {
          if (const auto entering = distanceToCrosswalk(spline, id)) {
            crossings.emplace_back(id, entering.value());
          }
        }
      }
    }
    return crossings;
  };

  // Whether a vehicle is still in the way, measured to where its lane leaves the crossed one:
  // one already past that point is no longer in the way, and so is a standing one short of it.
  const auto is_approaching = [&](const auto & crossings, const lanelet::Id crossed_id) {
    for (const auto & [id, entering] : crossings) {
      if (
        const auto crossing_s =
          lanelet_wrapper::lanelet_map::centerPointsSpline(id)->getCollisionPointIn2D(
            lanelet_wrapper::lanelet_map::laneletPolygon(crossed_id), true)) {
        for (const auto & status : other_statuses) {
          if (const auto & pose = status.getCanonicalizedLaneletPose()) {
            if (const auto distance = lanelet_wrapper::distance::longitudinalDistance(
                  static_cast<LaneletPose>(pose.value()),
                  helper::constructLaneletPose(id, crossing_s.value(), 0.0));
                distance and distance.value() <= status.getTwist().linear.x * required_gap) {
              return true;
            }
          }
        }
      }
    }
    return false;
  };

  // The nearest distance the vehicle can stop at with its body out of a lane it gives way to.
  auto min_distance = std::optional<double>();
  for (const auto & lanelet_id : following_lanelets) {
    if (const auto crossings = crossing_lane_ids(lanelet_id);
        is_approaching(crossings, lanelet_id)) {
      for (const auto & [id, entering] : crossings) {
        // The line is in the lane at `entering`, so the gap to a distance the body is out of
        // it at is halved until the two agree.
        auto clear = 0.0, entered = entering;
        while (distance_accuracy < entered - clear) {
          const auto midpoint = (clear + entered) / 2.0;
          (enters(id, midpoint) ? entered : clear) = midpoint;
        }
        min_distance = std::min(min_distance.value_or(clear), clear);
      }
    }
  }
  return min_distance;
}

auto distanceToNearestConflictingPose(
  const lanelet::Ids & following_lanelets, const math::geometry::CatmullRomSplineInterface & spline,
  const CanonicalizedEntityStatus & from_status,
  const std::vector<CanonicalizedEntityStatus> & other_statuses) -> std::optional<double>
{
  if (not from_status.isInLanelet()) {
    return std::nullopt;
  }

  auto conflicting_entities_on_lanelets =
    [&other_statuses](
      const lanelet::Ids & conflicting_ids) -> std::vector<CanonicalizedEntityStatus> {
    std::vector<CanonicalizedEntityStatus> conflicting_entity_status;
    for (const auto & status : other_statuses) {
      if (
        status.isInLanelet() &&
        std::find(conflicting_ids.cbegin(), conflicting_ids.cend(), status.getLaneletId()) !=
          conflicting_ids.cend()) {
        conflicting_entity_status.push_back(status);
      }
    }
    return conflicting_entity_status;
  };

  std::optional<double> min_distance = std::nullopt;
  auto try_min_distance = [&min_distance](const double & distance) {
    if (not min_distance.has_value() or distance < min_distance.value()) {
      min_distance = distance;
    }
  };

  const auto & conflicting_entities_on_crosswalk = conflicting_entities_on_lanelets(
    lanelet_wrapper::lanelet_map::conflictingCrosswalkIds(following_lanelets));

  for (const auto & status : conflicting_entities_on_crosswalk) {
    if (const auto & pose = status.getCanonicalizedLaneletPose()) {
      if (const auto s = distanceToCrosswalk(spline, pose->getLaneletId())) {
        try_min_distance(s.value());
      }
    }
  }

  const auto & conflicting_entities_on_lane = conflicting_entities_on_lanelets(
    lanelet_wrapper::lanelet_map::conflictingLaneIds(following_lanelets));

  for (const auto & status : conflicting_entities_on_lane) {
    if (const auto & pose = status.getCanonicalizedLaneletPose()) {
      if (
        const auto s = splineDistanceToBoundingBox(
          spline, from_status.getCanonicalizedLaneletPose().value(), from_status.getBoundingBox(),
          pose.value(), status.getBoundingBox())) {
        try_min_distance(s.value());
      }
    }
  }

  if (
    const auto distance =
      distanceToCrossingTraffic(following_lanelets, spline, from_status, other_statuses)) {
    try_min_distance(distance.value());
  }

  return min_distance;
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
