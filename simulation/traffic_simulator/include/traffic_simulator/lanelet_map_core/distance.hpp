
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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_DISTANCE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_DISTANCE_HPP_

#include <lanelet2_core/geometry/Lanelet.h>

#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/spline/catmull_rom_spline_interface.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace lanelet_map_core
{
namespace distance
{
using Point = geometry_msgs::msg::Point;
using Spline = math::geometry::CatmullRomSpline;
using SplineInterface = math::geometry::CatmullRomSplineInterface;
using LaneletPose = traffic_simulator_msgs::msg::LaneletPose;

auto getLateralDistance(
  const LaneletPose & from, const LaneletPose & to, const bool allow_lane_change)
  -> std::optional<double>;

auto getLongitudinalDistance(
  const LaneletPose & from, const LaneletPose & to, const bool allow_lane_change)
  -> std::optional<double>;

// StopLine
auto getDistanceToStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<Point> & route_waypoints)
  -> std::optional<double>;

auto getDistanceToStopLine(const lanelet::Ids & route_lanelets, const SplineInterface & spline)
  -> std::optional<double>;

auto getDistanceToStopLine(
  const std::vector<Point> & route_waypoints, const lanelet::Id stop_line_id)
  -> std::optional<double>;

// TrafficLigthStopLine
auto getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<Point> & waypoints)
  -> std::optional<double>;

auto getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const SplineInterface & spline) -> std::optional<double>;

auto getDistanceToTrafficLightStopLine(
  const std::vector<Point> & route_waypoints, const lanelet::Id traffic_light_id)
  -> std::optional<double>;

auto getDistanceToTrafficLightStopLine(
  const SplineInterface & route_spline, const lanelet::Id traffic_light_id)
  -> std::optional<double>;

// Crosswalk
auto distanceToCrosswalk(const std::vector<Point> & route_waypoints, const lanelet::Id crosswalk_id)
  -> std::optional<double>;

auto distanceToCrosswalk(const SplineInterface & route_spline, const lanelet::Id crosswalk_id)
  -> std::optional<double>;

// private
namespace
{
auto getStopLinesOnPath(const lanelet::Ids & lanelet_ids) -> lanelet::ConstLineStrings3d;
}
}  // namespace distance
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_DISTANCE_HPP_