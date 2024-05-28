
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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_DISTANCE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_DISTANCE_HPP_

#include <lanelet2_core/geometry/Lanelet.h>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>
#include <lanelet_map/lanelet_map.hpp>

namespace traffic_simulator
{
namespace lanelet2
{
namespace distance
{
auto getLateralDistance(
  const traffic_simulator_msgs::msg::LaneletPose & from,
  const traffic_simulator_msgs::msg::LaneletPose & to, bool allow_lane_change)
  -> std::optional<double>;

auto getLongitudinalDistance(
  const traffic_simulator_msgs::msg::LaneletPose & from,
  const traffic_simulator_msgs::msg::LaneletPose & to, bool allow_lane_change = false)
  -> std::optional<double>;

auto getDistanceToStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<geometry_msgs::msg::Point> & waypoints)
  -> std::optional<double>;

auto getDistanceToStopLine(
  const lanelet::Ids & route_lanelets, const math::geometry::CatmullRomSplineInterface & spline)
  -> std::optional<double>;

auto getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<geometry_msgs::msg::Point> & waypoints)
  -> std::optional<double>;

auto getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const math::geometry::CatmullRomSplineInterface & spline)
  -> std::optional<double>;

auto getDistanceToTrafficLightStopLine(
  const std::vector<geometry_msgs::msg::Point> & waypoints, const lanelet::Id traffic_light_id)
  -> std::optional<double>;

auto getDistanceToTrafficLightStopLine(
  const math::geometry::CatmullRomSplineInterface & spline, const lanelet::Id traffic_light_id)
  -> std::optional<double>;

// private for distance namespace
namespace
{
auto getStopLinesOnPath(const lanelet::Ids & lanelet_ids) -> lanelet::ConstLineStrings3d;
}
}  // namespace distance
}  // namespace lanelet2
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_DISTANCE_HPP_
