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
#include <traffic_simulator/lanelet_wrapper/lane_change.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/lanelet_wrapper/route.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace distance
{
auto lateralDistance(const LaneletPose & from, const LaneletPose & to, const bool allow_lane_change)
  -> std::optional<double>
{
  // route must exist for lateral distance to be calculated at all
  if (const auto route = route::route(from.lanelet_id, to.lanelet_id, allow_lane_change);
      route.empty()) {
    return std::nullopt;
  } else if (not allow_lane_change) {
    return to.offset - from.offset;
  } else {
    constexpr double matching_distance{10.0};
    double lateral_distance_by_lane_change{0.0};
    for (std::size_t i = 0; i < route.size() - 1; ++i) {
      // if there is a lane change
      if (lane_change::canChangeLane(route[i], route[i + 1])) {
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
  const LaneletPose & from, const LaneletPose & to, bool const allow_lane_change)
  -> std::optional<double>
{
  if (from.lanelet_id == to.lanelet_id) {
    return from.s > to.s ? std::nullopt : std::make_optional(to.s - from.s);
  } else if (const auto route = route::route(from.lanelet_id, to.lanelet_id, allow_lane_change);
             route.empty()) {
    return std::nullopt;
  } else {
    constexpr double matching_distance{10.0};
    double longitudinal_distance{0.0};
    /// @note in this for loop, some cases are marked by @note command. each case is explained in the document.
    /// @sa https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/DistanceCalculation/
    for (std::size_t i = 0; i < route.size(); i++) {
      // if there is a lane change and it is not last step in route
      if (
        allow_lane_change && i < route.size() - 1 &&
        lane_change::canChangeLane(route[i], route[i + 1])) {
        /// @note "the lanelet before the lane change" case
        const auto next_lanelet_pose = helper::constructLaneletPose(route[i + 1], 0.0, 0.0);
        const auto current_lanelet_pose = helper::constructLaneletPose(route[i], 0.0, 0.0);
        if (
          auto next_lanelet_origin_from_current_lanelet =
            pose::toLaneletPose(pose::toMapPose(next_lanelet_pose).pose, route[i], 10.0)) {
          longitudinal_distance += next_lanelet_origin_from_current_lanelet->s;
        } else if (
          auto current_lanelet_origin_from_next_lanelet =
            pose::toLaneletPose(pose::toMapPose(current_lanelet_pose).pose, route[i + 1], 10.0)) {
          longitudinal_distance -= current_lanelet_origin_from_next_lanelet->s;
        } else {
          /// @todo maybe an exception should be thrown here? since the route exists but is incorrect?
          return std::nullopt;
        }

        /// @note "first lanelet before the lane change" case
        if (route[i] == from.lanelet_id) {
          longitudinal_distance += lanelet_map::laneletLength(route[i + 1]) - from.s;
          if (route[i + 1] == to.lanelet_id) {
            return longitudinal_distance -= lanelet_map::laneletLength(route[i + 1]) - to.s;
          }
        }

      } else {
        if (route[i] == from.lanelet_id) {
          /// @note "first lanelet" case
          longitudinal_distance = lanelet_map::laneletLength(from.lanelet_id) - from.s;
        } else if (route[i] == to.lanelet_id) {
          /// @note "last lanelet" case
          longitudinal_distance += to.s;
        } else {
          ///@note "normal intermediate lanelet" case
          longitudinal_distance += lanelet_map::laneletLength(route[i]);
        }
      }
    }
    return longitudinal_distance;
  }
}

// StopLine
auto distanceToStopLine(const lanelet::Ids & route_lanelets, const SplineInterface & route_spline)
  -> std::optional<double>
{
  if (route_spline.getLength() <= 0) {
    return std::nullopt;
  } else {
    auto stopLinesOnPath = [](const lanelet::Ids & lanelet_ids) -> lanelet::ConstLineStrings3d {
      lanelet::ConstLineStrings3d stop_lines;
      for (const auto & traffic_sign : traffic_lights::trafficSignsOnPath(lanelet_ids)) {
        if (traffic_sign->type() == "stop_sign") {
          const auto & ref_lines = traffic_sign->refLines();
          stop_lines.insert(stop_lines.end(), ref_lines.begin(), ref_lines.end());
        }
      }
      return stop_lines;
    };

    std::vector<double> collision_points;
    // fill in collision_points using stop_lines
    const auto stop_lines = stopLinesOnPath({route_lanelets});
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
  if (auto traffic_light_ids = traffic_lights::trafficLightIdsOnPath(route_lanelets);
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

// crosswalk
auto distanceToCrosswalk(const std::vector<Point> & route_waypoints, const lanelet::Id crosswalk_id)
  -> std::optional<double>
{
  if (route_waypoints.empty()) {
    return std::nullopt;
  } else {
    const Spline route_spline(route_waypoints);
    return route_spline.getCollisionPointIn2D(
      lanelet_wrapper::lanelet_map::laneletPolygon(crosswalk_id));
  }
}

auto distanceToCrosswalk(const SplineInterface & route_spline, const lanelet::Id crosswalk_id)
  -> std::optional<double>
{
  return route_spline.getCollisionPointIn2D(
    lanelet_wrapper::lanelet_map::laneletPolygon(crosswalk_id), false);
}
}  // namespace distance
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
