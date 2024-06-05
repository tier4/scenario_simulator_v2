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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_LANELET_MAP_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_LANELET_MAP_HPP_

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace lanelet_core
{
namespace lanelet_map
{
using Point = geometry_msgs::msg::Point;
using Spline = math::geometry::CatmullRomSpline;

auto isInLanelet(const double s, const lanelet::Id lanelet_id) -> bool;

auto isInLanelet(const Point point, const lanelet::Id lanelet_id) -> bool;

auto getLaneletLength(const lanelet::Id lanelet_id) -> double;

auto getLaneletIds() -> lanelet::Ids;

auto getNearbyLaneletIds(
  const Point &, const double distance_threshold, const bool include_crosswalk,
  const std::size_t search_count) -> lanelet::Ids;

// Center points
auto getCenterPoints(const lanelet::Ids & lanelet_ids) -> std::vector<Point>;

auto getCenterPoints(const lanelet::Id lanelet_id) -> std::vector<Point>;

auto getCenterPointsSpline(const lanelet::Id lanelet_id) -> std::shared_ptr<Spline>;

// Next lanelet
auto getNextLaneletIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids;

auto getNextLaneletIds(const lanelet::Ids & lanelet_ids, const std::string & turn_direction)
  -> lanelet::Ids;

auto getNextLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids;

auto getNextLaneletIds(const lanelet::Id lanelet_id, const std::string & turn_direction)
  -> lanelet::Ids;

//Previous lanelet
auto getPreviousLaneletIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids;

auto getPreviousLaneletIds(const lanelet::Ids & lanelet_ids, const std::string & turn_direction)
  -> lanelet::Ids;

auto getPreviousLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids;

auto getPreviousLaneletIds(const lanelet::Id lanelet_id, const std::string & turn_direction)
  -> lanelet::Ids;

//Bounds
auto getLeftBound(const lanelet::Id lanelet_id) -> std::vector<Point>;

auto getRightBound(const lanelet::Id lanelet_id) -> std::vector<Point>;

// Polygons
auto getLaneletPolygon(const lanelet::Id lanelet_id) -> std::vector<Point>;

auto getStopLinePolygon(const lanelet::Id lanelet_id) -> std::vector<Point>;

// Relations
auto getRightOfWayLaneletIds(const lanelet::Ids & lanelet_ids)
  -> std::unordered_map<lanelet::Id, lanelet::Ids>;

auto getRightOfWayLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids;

auto getConflictingLaneIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids;

auto getConflictingCrosswalkIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids;

// private
namespace
{
auto toPolygon(const lanelet::ConstLineString3d & line_string) -> std::vector<Point>;

auto excludeSubtypeLanelets(
  const std::vector<std::pair<double, lanelet::Lanelet>> & pair, const char subtype[])
  -> std::vector<std::pair<double, lanelet::Lanelet>>;
}  // namespace
}  // namespace lanelet_map
}  // namespace lanelet_core
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_LANELET_MAP_HPP_
