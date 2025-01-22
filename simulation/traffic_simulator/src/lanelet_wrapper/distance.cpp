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

#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/distance.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/lanelet_wrapper/route.hpp>
#include <traffic_simulator/lanelet_wrapper/traffic_lights.hpp>

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

auto longitudinalDistance(
  const LaneletPose & from_pose, const LaneletPose & to_pose,
  const RoutingConfiguration & routing_configuration) -> std::optional<double>
{
  const auto is_lane_change_required = [&routing_configuration](
                                         const lanelet::Id current_lanelet,
                                         const lanelet::Id next_lanelet) -> bool {
    const auto next_lanelet_ids =
      lanelet_map::nextLaneletIds(current_lanelet, routing_configuration.routing_graph_type);
    return std::none_of(
      next_lanelet_ids.cbegin(), next_lanelet_ids.cend(),
      [next_lanelet](const lanelet::Id id) { return id == next_lanelet; });
  };

  /// @sa https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/DistanceCalculation/
  if (const auto route =
        route::route(from_pose.lanelet_id, to_pose.lanelet_id, routing_configuration);
      not route.empty() || from_pose.lanelet_id == to_pose.lanelet_id) {
    /// @note horizontal bar must intersect with the lanelet spline, so the distance was set arbitrarily to 10 meters.
    static constexpr double matching_distance = 10.0;
    double accumulated_distance = 0.0;
    // accumulate lanelet lengths alongside the route, considering possible lane changes
    for (std::size_t i = 1UL; i < route.size(); ++i) {
      // if lane change is required, add the distance traveled during the lane change
      // if lane change is not required, add the current lanelet length
      if (is_lane_change_required(route[i - 1UL], route[i])) {
        const auto current_lanelet_spline = lanelet_map::centerPointsSpline(route[i - 1UL]);
        const auto next_lanelet_spline = lanelet_map::centerPointsSpline(route[i]);

        // first, lanelets are matched at the start (s = 0.0) of each lanelet; only if this fails,
        // matching is performed at a larger s value, which should cover the rest of the cases.
        static constexpr double lanelet_starting_s = 0.0;
        const auto lanelet_min_middle_s =
          std::min(current_lanelet_spline->getLength(), next_lanelet_spline->getLength()) * 0.5;

        const auto current_start_matching_s = current_lanelet_spline->getSValue(
          next_lanelet_spline->getPose(lanelet_starting_s), matching_distance);
        const auto next_start_matching_s = next_lanelet_spline->getSValue(
          current_lanelet_spline->getPose(lanelet_starting_s), matching_distance);
        const auto current_middle_matching_s = current_lanelet_spline->getSValue(
          next_lanelet_spline->getPose(lanelet_min_middle_s), matching_distance);
        const auto next_middle_matching_s = next_lanelet_spline->getSValue(
          current_lanelet_spline->getPose(lanelet_min_middle_s), matching_distance);

        if (current_start_matching_s.has_value()) {
          accumulated_distance += current_start_matching_s.value();
        } else if (next_start_matching_s.has_value()) {
          accumulated_distance -= next_start_matching_s.value();
        } else if (current_middle_matching_s.has_value()) {
          accumulated_distance += current_middle_matching_s.value() - lanelet_min_middle_s;
        } else if (next_middle_matching_s.has_value()) {
          accumulated_distance -= next_middle_matching_s.value() - lanelet_min_middle_s;
        } else {
          return std::nullopt;
        }
      } else {
        accumulated_distance += lanelet_map::laneletLength(route[i - 1UL]);
      }
    }
    // subtract the distance already traveled on the first lanelet: from_pose.s
    // and add the distance that needs to be traveled on the last: to_pose.s.
    if (const double distance = accumulated_distance - from_pose.s + to_pose.s; distance >= 0.0) {
      return std::make_optional(distance);
    } else {
      return std::nullopt;
    }
  } else {
    return std::nullopt;
  }
}

// StopLine
auto distanceToStopLine(const lanelet::Ids & route_lanelets, const SplineInterface & route_spline)
  -> std::optional<double>
{
  if (route_spline.getLength() <= 0) {
    return std::nullopt;
  } else {
    std::vector<double> collision_points;
    // fill in collision_points using stop_lines
    const auto stop_lines = lanelet_wrapper::lanelet_map::stopLinesOnPath({route_lanelets});
    for (const auto & stop_line : stop_lines) {
      std::vector<Point> stop_line_points;
      for (const auto & point : stop_line) {
        stop_line_points.emplace_back(
          geometry_msgs::build<geometry_msgs::msg::Point>().x(point.x()).y(point.y()).z(point.z()));
      }
      if (const auto collision_point = route_spline.getCollisionPointIn2D(stop_line_points)) {
        collision_points.push_back(collision_point.value());
      }
    }
    return collision_points.empty()
             ? std::nullopt
             : std::optional(*std::min_element(collision_points.begin(), collision_points.end()));
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

// TrafficLigthStopLine
auto distanceToTrafficLightStopLine(
  const SplineInterface & route_spline, const lanelet::Id traffic_light_id) -> std::optional<double>
{
  if (route_spline.getLength() <= 0) {
    return std::nullopt;
  } else {
    const auto stop_lines = traffic_lights::trafficLightStopLinesPoints(traffic_light_id);
    for (const auto & stop_line : stop_lines) {
      if (const auto collision_point = route_spline.getCollisionPointIn2D(stop_line)) {
        return collision_point;
      }
    }
    return std::nullopt;
  }
}

auto distanceToTrafficLightStopLine(
  const std::vector<Point> & route_waypoints, const lanelet::Id traffic_light_id)
  -> std::optional<double>
{
  if (route_waypoints.empty()) {
    return std::nullopt;
  } else {
    return distanceToTrafficLightStopLine(Spline{route_waypoints}, traffic_light_id);
  }
}

auto distanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const SplineInterface & route_spline)
  -> std::optional<double>
{
  if (auto traffic_light_ids =
        lanelet_wrapper::traffic_lights::trafficLightIdsOnPath(route_lanelets);
      traffic_light_ids.empty()) {
    return std::nullopt;
  } else {
    std::vector<double> collision_points;
    for (const auto traffic_light_id : traffic_light_ids) {
      const auto collision_point = distanceToTrafficLightStopLine(route_spline, traffic_light_id);
      if (collision_point) {
        collision_points.push_back(collision_point.value());
      }
    }
    return collision_points.empty()
             ? std::nullopt
             : std::optional(*std::min_element(collision_points.begin(), collision_points.end()));
  }
}

auto distanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<Point> & route_waypoints)
  -> std::optional<double>
{
  return distanceToTrafficLightStopLine(route_lanelets, Spline{route_waypoints});
}

// Crosswalk
auto distanceToCrosswalk(const std::vector<Point> & route_waypoints, const lanelet::Id crosswalk_id)
  -> std::optional<double>
{
  if (route_waypoints.empty()) {
    return std::nullopt;
  } else {
    return distanceToCrosswalk(Spline{route_waypoints}, crosswalk_id);
  }
}

auto distanceToCrosswalk(const SplineInterface & route_spline, const lanelet::Id crosswalk_id)
  -> std::optional<double>
{
  constexpr bool search_in_backward_direction{false};
  return route_spline.getCollisionPointIn2D(
    lanelet_wrapper::lanelet_map::laneletPolygon(crosswalk_id), search_in_backward_direction);
}
}  // namespace distance
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
