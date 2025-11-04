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

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/normalize.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace lanelet_map
{
// Basics
auto isInLanelet(const lanelet::Id lanelet_id, const double lanelet_pose_s) -> bool
{
  return 0 <= lanelet_pose_s and lanelet_pose_s <= laneletLength(lanelet_id);
}

auto isInLanelet(const lanelet::Id lanelet_id, const Point point) -> bool
{
  return lanelet::geometry::inside(
    LaneletWrapper::map()->laneletLayer.get(lanelet_id), lanelet::BasicPoint2d(point.x, point.y));
}

auto laneletLength(const lanelet::Id lanelet_id) -> double
{
  return LaneletWrapper::laneletLengthCache().getLength(lanelet_id, LaneletWrapper::map());
}

auto laneletAltitude(
  const lanelet::Id & lanelet_id, const geometry_msgs::msg::Pose & pose,
  const double matching_distance) -> std::optional<double>
{
  if (const auto spline = centerPointsSpline(lanelet_id)) {
    if (const auto s = spline->getSValue(pose, matching_distance)) {
      return spline->getPoint(s.value()).z;
    }
  }
  return std::nullopt;
}

auto laneletIds() -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto & lanelet_layer = LaneletWrapper::map()->laneletLayer;
  ids.reserve(lanelet_layer.size());
  for (const auto & lanelet : lanelet_layer) {
    ids.push_back(lanelet.id());
  }
  return ids;
}

auto nearbyLaneletIds(
  const Point & point, const double distance_thresh, const bool include_crosswalk,
  const std::size_t search_count) -> lanelet::Ids
{
  auto isEmptyOrBeyondThreshold = [&distance_thresh](const auto & lanelets) {
    return lanelets.empty() || lanelets.front().first > distance_thresh;
  };

  auto excludeSubtypeLanelets =
    [](
      const std::vector<std::pair<double, lanelet::Lanelet>> & pair_distance_lanelet,
      const char subtype[]) {
      std::vector<std::pair<double, lanelet::Lanelet>> filtered_lanelets;
      for (const auto & pair : pair_distance_lanelet) {
        if (
          pair.second.hasAttribute(lanelet::AttributeName::Subtype) &&
          pair.second.attribute(lanelet::AttributeName::Subtype).value() != subtype) {
          filtered_lanelets.push_back(pair);
        }
      }
      return filtered_lanelets;
    };

  auto nearest_lanelets = lanelet::geometry::findNearest(
    LaneletWrapper::map()->laneletLayer, lanelet::BasicPoint2d(point.x, point.y),
    static_cast<unsigned>(search_count));

  /// @note check for current content, if not empty then optionally apply filter
  if (isEmptyOrBeyondThreshold(nearest_lanelets)) {
    return {};
  } else if (!include_crosswalk) {
    nearest_lanelets =
      excludeSubtypeLanelets(nearest_lanelets, lanelet::AttributeValueString::Crosswalk);
  }

  /// @note check again
  if (isEmptyOrBeyondThreshold(nearest_lanelets)) {
    return {};
  } else {
    lanelet::Ids target_lanelet_ids;
    for (const auto & [distance, lanelet] : nearest_lanelets) {
      if (distance <= distance_thresh) {
        target_lanelet_ids.push_back(lanelet.id());
      }
    }
    return target_lanelet_ids;
  }
}

// Center points
auto centerPoints(const lanelet::Ids & lanelet_ids) -> std::vector<Point>
{
  if (lanelet_ids.empty()) {
    return {};
  } else {
    std::vector<Point> center_points;
    for (const auto & lanelet_id : lanelet_ids) {
      const auto & points = centerPoints(lanelet_id);
      center_points.insert(center_points.end(), points.begin(), points.end());
    }
    /// @note We intentionally do not sort here, because only consecutive duplicates are supposed to be removed
    center_points.erase(
      std::unique(center_points.begin(), center_points.end()), center_points.end());
    return center_points;
  }
}

auto centerPoints(const lanelet::Id lanelet_id) -> std::vector<Point>
{
  return LaneletWrapper::centerPointsCache().getCenterPoints(lanelet_id, LaneletWrapper::map());
}

auto centerPointsSpline(const lanelet::Id lanelet_id) -> std::shared_ptr<Spline>
{
  return LaneletWrapper::centerPointsCache().getCenterPointsSpline(
    lanelet_id, LaneletWrapper::map());
}

// Next lanelet
auto nextLaneletIds(const lanelet::Id lanelet_id, const RoutingGraphType type) -> lanelet::Ids
{
  lanelet::Ids next_lanelet_ids;
  const auto & lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
  const auto & following_lanelets = LaneletWrapper::routingGraph(type)->following(lanelet);
  next_lanelet_ids.reserve(following_lanelets.size());
  for (const auto & following_lanelet : following_lanelets) {
    next_lanelet_ids.push_back(following_lanelet.id());
  }
  return next_lanelet_ids;
}

auto nextLaneletIds(const lanelet::Ids & lanelet_ids, const RoutingGraphType type) -> lanelet::Ids
{
  std::set<lanelet::Id> next_lanelet_ids_set;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto & next_lanelet_ids = nextLaneletIds(lanelet_id, type);
    next_lanelet_ids_set.insert(next_lanelet_ids.begin(), next_lanelet_ids.end());
  }
  return lanelet::Ids(next_lanelet_ids_set.begin(), next_lanelet_ids_set.end());
}

auto nextLaneletIds(
  const lanelet::Id lanelet_id, std::string_view turn_direction, const RoutingGraphType type)
  -> lanelet::Ids
{
  lanelet::Ids next_lanelet_ids;
  const auto & reference_lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
  for (const auto & following_lanelet :
       LaneletWrapper::routingGraph(type)->following(reference_lanelet)) {
    if (following_lanelet.attributeOr("turn_direction", "else") == turn_direction) {
      next_lanelet_ids.push_back(following_lanelet.id());
    }
  }
  return next_lanelet_ids;
}

auto nextLaneletIds(
  const lanelet::Ids & lanelet_ids, std::string_view turn_direction, const RoutingGraphType type)
  -> lanelet::Ids
{
  std::set<lanelet::Id> next_lanelet_ids_set;
  for (const auto & lanelet_id : lanelet_ids) {
    auto next_lanelet_ids = nextLaneletIds(lanelet_id, turn_direction, type);
    next_lanelet_ids_set.insert(next_lanelet_ids.begin(), next_lanelet_ids.end());
  }
  return lanelet::Ids(next_lanelet_ids_set.begin(), next_lanelet_ids_set.end());
}

// Previous lanelet
auto previousLaneletIds(const lanelet::Id lanelet_id, const RoutingGraphType type) -> lanelet::Ids
{
  lanelet::Ids previous_lanelet_ids;
  const auto & lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
  const auto & previous_lanelets = LaneletWrapper::routingGraph(type)->previous(lanelet);
  previous_lanelet_ids.reserve(previous_lanelets.size());
  for (const auto & previous_lanelet : previous_lanelets) {
    previous_lanelet_ids.push_back(previous_lanelet.id());
  }
  return previous_lanelet_ids;
}

auto previousLaneletIds(const lanelet::Ids & lanelet_ids, const RoutingGraphType type)
  -> lanelet::Ids
{
  std::set<lanelet::Id> previous_lanelet_ids_set;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto & previous_lanelet_ids = previousLaneletIds(lanelet_id, type);
    previous_lanelet_ids_set.insert(previous_lanelet_ids.begin(), previous_lanelet_ids.end());
  }
  return lanelet::Ids(previous_lanelet_ids_set.begin(), previous_lanelet_ids_set.end());
}

auto previousLaneletIds(
  const lanelet::Id lanelet_id, std::string_view turn_direction, const RoutingGraphType type)
  -> lanelet::Ids
{
  lanelet::Ids previous_lanelet_ids;
  const auto reference_lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
  for (const auto & previous_lanelet :
       LaneletWrapper::routingGraph(type)->previous(reference_lanelet)) {
    if (previous_lanelet.attributeOr("turn_direction", "else") == turn_direction) {
      previous_lanelet_ids.push_back(previous_lanelet.id());
    }
  }
  return previous_lanelet_ids;
}

auto previousLaneletIds(
  const lanelet::Ids & lanelet_ids, std::string_view turn_direction, const RoutingGraphType type)
  -> lanelet::Ids
{
  std::set<lanelet::Id> previous_lanelet_ids_set;
  for (const auto & lanelet_id : lanelet_ids) {
    auto previous_lanelet_ids = previousLaneletIds(lanelet_id, turn_direction, type);
    previous_lanelet_ids_set.insert(previous_lanelet_ids.begin(), previous_lanelet_ids.end());
  }
  return lanelet::Ids(previous_lanelet_ids_set.begin(), previous_lanelet_ids_set.end());
}

// Bounds
auto leftBound(const lanelet::Id lanelet_id) -> std::vector<Point>
{
  return toPolygon(LaneletWrapper::map()->laneletLayer.get(lanelet_id).leftBound());
}

auto rightBound(const lanelet::Id lanelet_id) -> std::vector<Point>
{
  return toPolygon(LaneletWrapper::map()->laneletLayer.get(lanelet_id).rightBound());
}

// Polygons
auto laneletPolygon(const lanelet::Id lanelet_id) -> std::vector<Point>
{
  std::vector<Point> points;
  const auto & lanelet_polygon = LaneletWrapper::map()->laneletLayer.get(lanelet_id).polygon3d();
  points.reserve(lanelet_polygon.size());
  for (const auto & point : lanelet_polygon) {
    points.push_back(geometry_msgs::build<Point>().x(point.x()).y(point.y()).z(point.z()));
  }
  return points;
}

auto stopLinePolygon(const lanelet::Id lanelet_id) -> std::vector<Point>
{
  /// @todo here you should probably add a verify if the passed lanelet_id is indeed a stop_line
  return toPolygon(LaneletWrapper::map()->lineStringLayer.get(lanelet_id));
}

auto toPolygon(const lanelet::ConstLineString3d & line_string) -> std::vector<Point>
{
  std::vector<Point> points;
  points.reserve(line_string.size());
  for (const auto & point : line_string) {
    points.push_back(geometry_msgs::build<Point>().x(point.x()).y(point.y()).z(point.z()));
  }
  return points;
}

// Relations
auto rightOfWayLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids right_of_way_lanelets_ids;
  const auto & right_of_ways =
    LaneletWrapper::map()->laneletLayer.get(lanelet_id).regulatoryElementsAs<lanelet::RightOfWay>();
  for (const auto & right_of_way : right_of_ways) {
    for (const auto & right_of_way_lanelet : right_of_way->rightOfWayLanelets()) {
      if (right_of_way_lanelet.id() != lanelet_id) {
        right_of_way_lanelets_ids.push_back(right_of_way_lanelet.id());
      }
    }
  }
  return right_of_way_lanelets_ids;
}

auto rightOfWayLaneletIds(const lanelet::Ids & lanelet_ids)
  -> std::unordered_map<lanelet::Id, lanelet::Ids>
{
  std::unordered_map<lanelet::Id, lanelet::Ids> right_of_way_lanelets_ids;
  for (const auto & lanelet_id : lanelet_ids) {
    right_of_way_lanelets_ids.try_emplace(lanelet_id, rightOfWayLaneletIds(lanelet_id));
  }
  return right_of_way_lanelets_ids;
}

auto conflictingCrosswalkIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  constexpr size_t routing_graph_id{1};
  constexpr double height_clearance{4};
  /// @note it is not clear if the distinction for crosswalks only is implemented here
  lanelet::Ids conflicting_crosswalk_ids;
  lanelet::routing::RoutingGraphContainer graphs_container(
    {LaneletWrapper::routingGraph(RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER),
     LaneletWrapper::routingGraph(RoutingGraphType::PEDESTRIAN)});
  for (const auto & lanelet_id : lanelet_ids) {
    const auto & conflicting_crosswalks = graphs_container.conflictingInGraph(
      LaneletWrapper::map()->laneletLayer.get(lanelet_id), routing_graph_id, height_clearance);
    for (const auto & conflicting_crosswalk : conflicting_crosswalks) {
      conflicting_crosswalk_ids.push_back(conflicting_crosswalk.id());
    }
  }
  return conflicting_crosswalk_ids;
}

// Objects on path
auto trafficSignsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>
{
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto & lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
    const auto & traffic_signs = lanelet.regulatoryElementsAs<const lanelet::TrafficSign>();
    for (const auto & traffic_sign : traffic_signs) {
      ret.push_back(traffic_sign);
    }
  }
  return ret;
}

auto trafficSigns() -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>
{
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> ret;
  for (const auto & lanelet_id : laneletIds()) {
    const auto lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
    const auto traffic_signs = lanelet.regulatoryElementsAs<const lanelet::TrafficSign>();
    for (const auto & traffic_sign : traffic_signs) {
      ret.push_back(traffic_sign);
    }
  }
  return ret;
}

auto stopLines() -> lanelet::ConstLineStrings3d
{
  lanelet::ConstLineStrings3d stop_lines;
  for (const auto & traffic_sign : lanelet_wrapper::lanelet_map::trafficSigns()) {
    if (traffic_sign->type() == "stop_sign") {
      const auto & ref_lines = traffic_sign->refLines();
      stop_lines.insert(stop_lines.end(), ref_lines.begin(), ref_lines.end());
    }
  }
  return stop_lines;
}

auto stopLinesOnPath(const lanelet::Ids & lanelet_ids) -> lanelet::ConstLineStrings3d
{
  lanelet::ConstLineStrings3d stop_lines;
  for (const auto & traffic_sign : lanelet_wrapper::lanelet_map::trafficSignsOnPath(lanelet_ids)) {
    if (traffic_sign->type() == "stop_sign") {
      const auto & ref_lines = traffic_sign->refLines();
      stop_lines.insert(stop_lines.end(), ref_lines.begin(), ref_lines.end());
    }
  }
  return stop_lines;
}

auto stopLineIds() -> lanelet::Ids
{
  lanelet::Ids stop_line_ids;
  const auto & stop_lines = stopLines();
  stop_line_ids.reserve(stop_lines.size());
  for (const auto & ret : stop_lines) {
    stop_line_ids.push_back(ret.id());
  }
  return stop_line_ids;
}

auto stopLineIdsOnPath(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids stop_line_ids;
  const auto & stop_lines = stopLinesOnPath(lanelet_ids);
  stop_line_ids.reserve(stop_lines.size());
  for (const auto & ret : stop_lines) {
    stop_line_ids.push_back(ret.id());
  }
  return stop_line_ids;
}
}  // namespace lanelet_map
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
