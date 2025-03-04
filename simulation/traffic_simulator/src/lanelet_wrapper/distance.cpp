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
