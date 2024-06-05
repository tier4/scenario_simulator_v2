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

#include <traffic_simulator/lanelet_map_core/distance.hpp>
#include <traffic_simulator/lanelet_map_core/lanelet_map.hpp>
#include <traffic_simulator/lanelet_map_core/pose.hpp>
#include <traffic_simulator/lanelet_map_core/route.hpp>

namespace traffic_simulator
{
namespace lanelet_map_core
{
namespace distance
{
auto getLateralDistance(
  const LaneletPose & from, const LaneletPose & to, const bool allow_lane_change)
  -> std::optional<double>
{
  const auto route = route::getRoute(from.lanelet_id, to.lanelet_id, allow_lane_change);
  if (route.empty()) {
    return std::nullopt;
  }
  if (allow_lane_change) {
    double lateral_distance_by_lane_change = 0.0;
    for (unsigned int i = 0; i < route.size() - 1; i++) {
      auto next_lanelet_ids = lanelet_map::getNextLaneletIds(route[i]);
      if (auto next_lanelet = std::find_if(
            next_lanelet_ids.begin(), next_lanelet_ids.end(),
            [&route, i](const lanelet::Id id) { return id == route[i + 1]; });
          next_lanelet == next_lanelet_ids.end()) {
        LaneletPose next_lanelet_pose;
        next_lanelet_pose.lanelet_id = route[i + 1];
        next_lanelet_pose.s = 0.0;
        next_lanelet_pose.offset = 0.0;

        if (
          auto next_lanelet_origin_from_current_lanelet =
            pose::toLaneletPose(pose::toMapPose(next_lanelet_pose).pose, route[i], 10.0)) {
          lateral_distance_by_lane_change += next_lanelet_origin_from_current_lanelet->offset;
        } else {
          LaneletPose current_lanelet_pose = next_lanelet_pose;
          current_lanelet_pose.lanelet_id = route[i];
          if (
            auto current_lanelet_origin_from_next_lanelet =
              pose::toLaneletPose(pose::toMapPose(current_lanelet_pose).pose, route[i + 1], 10.0)) {
            lateral_distance_by_lane_change -= current_lanelet_origin_from_next_lanelet->offset;
          } else {
            return std::nullopt;
          }
        }
      }
    }
    return to.offset - from.offset + lateral_distance_by_lane_change;
  } else {
    return to.offset - from.offset;
  }
}

auto getLongitudinalDistance(
  const LaneletPose & from, const LaneletPose & to, bool const allow_lane_change)
  -> std::optional<double>
{
  if (from.lanelet_id == to.lanelet_id) {
    if (from.s > to.s) {
      return std::nullopt;
    } else {
      return to.s - from.s;
    }
  }
  const auto route = route::getRoute(from.lanelet_id, to.lanelet_id, allow_lane_change);
  if (route.empty()) {
    return std::nullopt;
  }
  double distance = 0;

  auto with_lane_change = [](
                            const bool allow_lane_change, const lanelet::Id current_lanelet,
                            const lanelet::Id next_lanelet) -> bool {
    if (allow_lane_change) {
      auto next_lanelet_ids = lanelet_map::getNextLaneletIds(current_lanelet);
      auto next_lanelet_itr = std::find_if(
        next_lanelet_ids.begin(), next_lanelet_ids.end(),
        [next_lanelet](const lanelet::Id id) { return id == next_lanelet; });
      return next_lanelet_itr == next_lanelet_ids.end();
    } else {
      return false;
    }
  };

  /// @note in this for loop, some cases are marked by @note command. each case is explained in the document.
  /// @sa https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/DistanceCalculation/
  for (unsigned int i = 0; i < route.size(); i++) {
    if (i < route.size() - 1 && with_lane_change(allow_lane_change, route[i], route[i + 1])) {
      /// @note "the lanelet before the lane change" case
      LaneletPose next_lanelet_pose;
      next_lanelet_pose.lanelet_id = route[i + 1];
      next_lanelet_pose.s = 0.0;
      next_lanelet_pose.offset = 0.0;

      if (
        auto next_lanelet_origin_from_current_lanelet =
          pose::toLaneletPose(pose::toMapPose(next_lanelet_pose).pose, route[i], 10.0)) {
        distance += next_lanelet_origin_from_current_lanelet->s;
      } else {
        LaneletPose current_lanelet_pose = next_lanelet_pose;
        current_lanelet_pose.lanelet_id = route[i];
        if (
          auto current_lanelet_origin_from_next_lanelet =
            pose::toLaneletPose(pose::toMapPose(current_lanelet_pose).pose, route[i + 1], 10.0)) {
          distance -= current_lanelet_origin_from_next_lanelet->s;
        } else {
          return std::nullopt;
        }
      }

      /// @note "first lanelet before the lane change" case
      if (route[i] == from.lanelet_id) {
        distance += lanelet_map::getLaneletLength(route[i + 1]) - from.s;
        if (route[i + 1] == to.lanelet_id) {
          distance -= lanelet_map::getLaneletLength(route[i + 1]) - to.s;
          return distance;
        }
      }
    } else {
      if (route[i] == from.lanelet_id) {
        /// @note "first lanelet" case
        distance = lanelet_map::getLaneletLength(from.lanelet_id) - from.s;
      } else if (route[i] == to.lanelet_id) {
        /// @note "last lanelet" case
        distance += to.s;
      } else {
        ///@note "normal intermediate lanelet" case
        distance += lanelet_map::getLaneletLength(route[i]);
      }
    }
  }
  return distance;
}

// StopLine
auto getDistanceToStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<Point> & route_waypoints)
  -> std::optional<double>
{
  if (route_waypoints.empty()) {
    return std::nullopt;
  }
  std::set<double> collision_points;
  if (route_waypoints.empty()) {
    return std::nullopt;
  }
  Spline spline(route_waypoints);
  const auto stop_lines = getStopLinesOnPath({route_lanelets});
  for (const auto & stop_line : stop_lines) {
    std::vector<Point> stop_line_points;
    for (const auto & point : stop_line) {
      Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      stop_line_points.emplace_back(p);
    }
    const auto collision_point = spline.getCollisionPointIn2D(stop_line_points);
    if (collision_point) {
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto getDistanceToStopLine(
  const lanelet::Ids & route_lanelets, const SplineInterface & route_spline)
  -> std::optional<double>
{
  if (route_spline.getLength() <= 0) {
    return std::nullopt;
  }
  std::set<double> collision_points;
  const auto stop_lines = getStopLinesOnPath({route_lanelets});
  for (const auto & stop_line : stop_lines) {
    std::vector<Point> stop_line_points;
    for (const auto & point : stop_line) {
      Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      stop_line_points.emplace_back(p);
    }
    const auto collision_point = route_spline.getCollisionPointIn2D(stop_line_points);
    if (collision_point) {
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto getDistanceToStopLine(
  const std::vector<Point> & route_waypoints, const lanelet::Id stop_line_id)
  -> std::optional<double>
{
  if (route_waypoints.empty()) {
    return std::nullopt;
  } else {
    const Spline spline(route_waypoints);
    const auto polygon = lanelet_map_core::lanelet_map::getStopLinePolygon(stop_line_id);
    return spline.getCollisionPointIn2D(polygon);
  }
}

// TrafficLigthStopLine
auto getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<Point> & waypoints)
  -> std::optional<double>
{
  auto traffic_light_ids = traffic_lights::getTrafficLightIdsOnPath(route_lanelets);
  if (traffic_light_ids.empty()) {
    return std::nullopt;
  }
  std::set<double> collision_points;
  for (const auto id : traffic_light_ids) {
    const auto collision_point = getDistanceToTrafficLightStopLine(waypoints, id);
    if (collision_point) {
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const SplineInterface & spline) -> std::optional<double>
{
  auto traffic_light_ids = traffic_lights::getTrafficLightIdsOnPath(route_lanelets);
  if (traffic_light_ids.empty()) {
    return std::nullopt;
  }
  std::set<double> collision_points;
  for (const auto id : traffic_light_ids) {
    const auto collision_point = getDistanceToTrafficLightStopLine(spline, id);
    if (collision_point) {
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto getDistanceToTrafficLightStopLine(
  const std::vector<Point> & route_waypoints, const lanelet::Id traffic_light_id)
  -> std::optional<double>
{
  if (route_waypoints.empty()) {
    return std::nullopt;
  }
  Spline spline(route_waypoints);
  const auto stop_lines = traffic_lights::getTrafficLightStopLinesPoints(traffic_light_id);
  for (const auto & stop_line : stop_lines) {
    const auto collision_point = spline.getCollisionPointIn2D(stop_line);
    if (collision_point) {
      return collision_point;
    }
  }
  return std::nullopt;
}

auto getDistanceToTrafficLightStopLine(
  const SplineInterface & route_spline, const lanelet::Id traffic_light_id) -> std::optional<double>
{
  if (route_spline.getLength() <= 0) {
    return std::nullopt;
  }
  const auto stop_lines = traffic_lights::getTrafficLightStopLinesPoints(traffic_light_id);
  for (const auto & stop_line : stop_lines) {
    const auto collision_point = route_spline.getCollisionPointIn2D(stop_line);
    if (collision_point) {
      return collision_point;
    }
  }
  return std::nullopt;
}

// crosswalk
auto distanceToCrosswalk(const std::vector<Point> & route_waypoints, const lanelet::Id crosswalk_id)
  -> std::optional<double>
{
  if (route_waypoints.empty()) {
    return std::nullopt;
  } else {
    Spline spline(route_waypoints);
    return spline.getCollisionPointIn2D(
      lanelet_map_core::lanelet_map::getLaneletPolygon(crosswalk_id));
  }
}

auto distanceToCrosswalk(const SplineInterface & route_spline, const lanelet::Id crosswalk_id)
  -> std::optional<double>
{
  return route_spline.getCollisionPointIn2D(
    lanelet_map_core::lanelet_map::getLaneletPolygon(crosswalk_id), false);
}
}  // namespace distance
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
