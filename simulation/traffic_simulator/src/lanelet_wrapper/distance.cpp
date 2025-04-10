// Copyright 2015 Tier IV, Inc. All rights reserved.
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

#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/distance.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/lanelet_wrapper/route.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace distance
{
auto lateralDistance(
  const LaneletPose & from, const LaneletPose & to,
  const RoutingConfiguration & routing_configuration) -> std::optional<double>
{
  const auto route = route::route(from.lanelet_id, to.lanelet_id, routing_configuration);
  if (route.empty()) {
    return std::nullopt;
  } else if (not routing_configuration.allow_lane_change) {
    return to.offset - from.offset;
  } else {
    constexpr double matching_distance{10.0};
    double lateral_distance_by_lane_change = 0.0;
    for (std::size_t i = 0; i < route.size() - 1; i++) {
      auto next_lanelet_ids =
        lanelet_map::nextLaneletIds(route[i], routing_configuration.routing_graph_type);
      if (auto next_lanelet = std::find_if(
            next_lanelet_ids.begin(), next_lanelet_ids.end(),
            [&route, i](const lanelet::Id & id) { return id == route[i + 1]; });
          next_lanelet == next_lanelet_ids.end()) {
        const auto current_lanelet_pose = helper::constructLaneletPose(route[i], 0.0, 0.0);
        const auto next_lanelet_pose = helper::constructLaneletPose(route[i + 1], 0.0, 0.0);
        if (
          const auto next_lanelet_origin_from_current_lanelet = pose::toLaneletPose(
            pose::toMapPose(next_lanelet_pose).pose, route[i], matching_distance)) {
          lateral_distance_by_lane_change += next_lanelet_origin_from_current_lanelet->offset;
        } else if (
          const auto current_lanelet_origin_from_next_lanelet = pose::toLaneletPose(
            pose::toMapPose(current_lanelet_pose).pose, route[i + 1], matching_distance)) {
          lateral_distance_by_lane_change -= current_lanelet_origin_from_next_lanelet->offset;
        } else {
          /// @todo maybe an exception should be thrown here? since the route exists but is incorrect?
          return std::nullopt;
        }
      }
    }
    return to.offset - from.offset + lateral_distance_by_lane_change;
  }
}

// StopLine
auto distanceToStopLine(const lanelet::Ids & route_lanelets, const SplineInterface & route_spline)
  -> std::optional<double>
{
  if (route_spline.getLength() <= 0.0) {
    return std::nullopt;
  } else {
    std::optional<double> min_distance{std::nullopt};
    const auto & stop_lines = lanelet_wrapper::lanelet_map::stopLinesOnPath({route_lanelets});
    for (const auto & stop_line : stop_lines) {
      const auto & stop_line_points = lanelet_wrapper::lanelet_map::toPolygon(stop_line);
      if (const auto & collision_point = route_spline.getCollisionPointIn2D(stop_line_points)) {
        if (not min_distance.has_value() or collision_point.value() < min_distance.value()) {
          min_distance = collision_point;
        }
      }
    }
    return min_distance;
  }
}

auto distanceToStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<Point> & route_waypoints)
  -> std::optional<double>
{
  if (route_waypoints.empty()) {
    return std::nullopt;
  } else {
    return distanceToStopLine(route_lanelets, Spline{route_waypoints});
  }
}

auto distanceToStopLine(const std::vector<Point> & route_waypoints, const lanelet::Id stop_line_id)
  -> std::optional<double>
{
  if (route_waypoints.empty()) {
    return std::nullopt;
  } else {
    const Spline route_spline{route_waypoints};
    return route_spline.getCollisionPointIn2D(
      lanelet_wrapper::lanelet_map::stopLinePolygon(stop_line_id));
  }
}
}  // namespace distance
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
