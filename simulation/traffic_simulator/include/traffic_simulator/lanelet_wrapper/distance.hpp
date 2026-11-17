
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

#ifndef TRAFFIC_SIMULATOR__LANELET_WRAPPER_DISTANCE_HPP_
#define TRAFFIC_SIMULATOR__LANELET_WRAPPER_DISTANCE_HPP_

#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace distance
{
auto lateralDistance(
  const LaneletPose & from, const LaneletPose & to,
  const RoutingConfiguration & routing_configuration = RoutingConfiguration())
  -> std::optional<double>;

auto longitudinalDistance(
  const LaneletPose & from, const LaneletPose & to,
  const RoutingConfiguration & routing_configuration = RoutingConfiguration())
  -> std::optional<double>;

// StopLine
auto distanceToStopLine(const lanelet::Ids & route_lanelets, const SplineInterface & route_spline)
  -> std::optional<double>;

auto distanceToStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<Point> & route_waypoints)
  -> std::optional<double>;

auto distanceToStopLine(const std::vector<Point> & route_waypoints, const lanelet::Id stop_line_id)
  -> std::optional<double>;

// TrafficLightStopLine
auto distanceToTrafficLightStopLine(
  const SplineInterface & route_spline, const lanelet::Id traffic_light_id)
  -> std::optional<double>;

auto distanceToTrafficLightStopLine(
  const std::vector<Point> & route_waypoints, const lanelet::Id traffic_light_id)
  -> std::optional<double>;

auto distanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const SplineInterface & route_spline)
  -> std::optional<double>;

auto distanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<Point> & route_waypoints)
  -> std::optional<double>;

// Crosswalk
auto distanceToCrosswalk(const std::vector<Point> & route_waypoints, const lanelet::Id crosswalk_id)
  -> std::optional<double>;

auto distanceToCrosswalk(const SplineInterface & route_spline, const lanelet::Id crosswalk_id)
  -> std::optional<double>;
}  // namespace distance
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__LANELET_WRAPPER_DISTANCE_HPP_
