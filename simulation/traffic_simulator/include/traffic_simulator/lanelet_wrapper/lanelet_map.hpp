// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__LANELET_WRAPPER_LANELET_MAP_HPP_
#define TRAFFIC_SIMULATOR__LANELET_WRAPPER_LANELET_MAP_HPP_

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace lanelet_map
{
auto isInLanelet(const lanelet::Id lanelet_id, const double lanelet_pose_s) -> bool;

auto isInLanelet(const lanelet::Id lanelet_id, const Point point) -> bool;

auto laneletLength(const lanelet::Id lanelet_id) -> double;

auto laneletAltitude(
  const lanelet::Id & lanelet_id, const geometry_msgs::msg::Pose & pose,
  const double matching_distance) -> std::optional<double>;

template <typename Lanelet>
auto laneletIds(const std::vector<Lanelet> & lanelets) -> lanelet::Ids
{
  lanelet::Ids ids;
  std::transform(
    lanelets.begin(), lanelets.end(), std::back_inserter(ids),
    [](const auto & lanelet) { return lanelet.id(); });
  return ids;
}

auto laneletIds() -> lanelet::Ids;

auto nearbyLaneletIds(
  const Point & point, const double distance_threshold, const bool include_crosswalk,
  const std::size_t search_count) -> lanelet::Ids;

auto centerPoints(const lanelet::Ids & lanelet_ids) -> std::vector<Point>;

auto centerPoints(const lanelet::Id lanelet_id) -> std::vector<Point>;

auto centerPointsSpline(const lanelet::Id lanelet_id) -> std::shared_ptr<Spline>;

auto nextLaneletIds(
  const lanelet::Id lanelet_id,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> lanelet::Ids;

auto nextLaneletIds(
  const lanelet::Ids & lanelet_ids,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> lanelet::Ids;

auto nextLaneletIds(
  const lanelet::Id lanelet_id, std::string_view turn_direction,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> lanelet::Ids;

auto nextLaneletIds(
  const lanelet::Ids & lanelet_ids, std::string_view turn_direction,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> lanelet::Ids;

auto previousLaneletIds(
  const lanelet::Id lanelet_id,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> lanelet::Ids;

auto previousLaneletIds(
  const lanelet::Ids & lanelet_ids,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> lanelet::Ids;

auto previousLaneletIds(
  const lanelet::Id lanelet_id, std::string_view turn_direction,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> lanelet::Ids;

auto previousLaneletIds(
  const lanelet::Ids & lanelet_ids, std::string_view turn_direction,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> lanelet::Ids;

// Bounds
auto leftBound(const lanelet::Id lanelet_id) -> std::vector<Point>;

auto rightBound(const lanelet::Id lanelet_id) -> std::vector<Point>;

// Polygons
auto stopLinePolygon(const lanelet::Id lanelet_id) -> std::vector<Point>;

auto toPolygon(const lanelet::ConstLineString3d & line_string) -> std::vector<Point>;

// Objects on path
auto trafficSignsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>;

auto stopLinesOnPath(const lanelet::Ids & lanelet_ids) -> lanelet::ConstLineStrings3d;

auto stopLineIdsOnPath(const lanelet::Ids & lanelet_ids) -> lanelet::Ids;
}  // namespace lanelet_map
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__LANELET_WRAPPER_LANELET_MAP_HPP_
