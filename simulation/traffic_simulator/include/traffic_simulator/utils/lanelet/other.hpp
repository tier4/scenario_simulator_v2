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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_OTHER_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_OTHER_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace lanelet2
{
namespace other
{
auto isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route) -> bool;

template <typename Lanelet>
auto getLaneletIds(const std::vector<Lanelet> & lanelets) -> lanelet::Ids
{
  lanelet::Ids ids;
  std::transform(
    lanelets.begin(), lanelets.end(), std::back_inserter(ids),
    [](const auto & lanelet) { return lanelet.id(); });
  return ids;
}

auto getLeftBound(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>;

auto getRightBound(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>;

auto getLaneletIds() -> lanelet::Ids;

auto getCenterPoints(const lanelet::Ids &) -> std::vector<geometry_msgs::msg::Point>;

auto getCenterPoints(const lanelet::Id) -> std::vector<geometry_msgs::msg::Point>;

auto getCenterPointsSpline(const lanelet::Id) -> std::shared_ptr<math::geometry::CatmullRomSpline>;

auto getLaneletLength(const lanelet::Id lanelet_id) -> double;

auto getNextLaneletIds(const lanelet::Ids &) -> lanelet::Ids;

auto getNextLaneletIds(const lanelet::Ids &, const std::string & turn_direction) -> lanelet::Ids;

auto getNextLaneletIds(const lanelet::Id) -> lanelet::Ids;

auto getNextLaneletIds(const lanelet::Id, const std::string & turn_direction) -> lanelet::Ids;

auto getPreviousLaneletIds(const lanelet::Ids &) -> lanelet::Ids;

auto getPreviousLaneletIds(const lanelet::Ids &, const std::string & turn_direction)
  -> lanelet::Ids;

auto getPreviousLaneletIds(const lanelet::Id) -> lanelet::Ids;

auto getPreviousLaneletIds(const lanelet::Id, const std::string & turn_direction) -> lanelet::Ids;

// Polygon and marker

auto getLaneletPolygon(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>;

auto getStopLinePolygon(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>;

auto generateMarker() -> visualization_msgs::msg::MarkerArray;

namespace
{
auto getNextRoadShoulderLanelet(const lanelet::Id) -> lanelet::Ids;

auto getPreviousRoadShoulderLanelet(const lanelet::Id) -> lanelet::Ids;

auto toPolygon(const lanelet::ConstLineString3d & line_string)
  -> std::vector<geometry_msgs::msg::Point>;

auto isInLanelet(const lanelet::Id lanelet_id, const double s) -> bool;

auto insertMarkerArray(
  visualization_msgs::msg::MarkerArray &, const visualization_msgs::msg::MarkerArray &) -> void;
}  // namespace
}  // namespace other
}  // namespace lanelet2
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_OTHER_HPP_