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
  for (const auto & lanelet : LaneletWrapper::map()->laneletLayer) {
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
        target_lanelet_ids.emplace_back(lanelet.id());
      }
    }
    return target_lanelet_ids;
  }
}

auto centerPoints(const lanelet::Ids & lanelet_ids) -> std::vector<Point>
{
  if (lanelet_ids.empty()) {
    return {};
  } else {
    std::vector<Point> center_points;
    for (const auto & lanelet_id : lanelet_ids) {
      auto points = centerPoints(lanelet_id);
      center_points.insert(center_points.end(), points.begin(), points.end());
    }
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

auto nextLaneletIds(const lanelet::Id lanelet_id, const RoutingGraphType type) -> lanelet::Ids
{
  lanelet::Ids next_lanelet_ids;
  const auto lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
  for (const auto & following_lanelet : LaneletWrapper::routingGraph(type)->following(lanelet)) {
    next_lanelet_ids.push_back(following_lanelet.id());
  }
  return next_lanelet_ids;
}

auto nextLaneletIds(const lanelet::Ids & lanelet_ids, const RoutingGraphType type) -> lanelet::Ids
{
  std::set<lanelet::Id> next_lanelet_ids_set;
  for (const auto & lanelet_id : lanelet_ids) {
    auto next_lanelet_ids = nextLaneletIds(lanelet_id, type);
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

auto previousLaneletIds(const lanelet::Id lanelet_id, const RoutingGraphType type) -> lanelet::Ids
{
  lanelet::Ids previous_lanelet_ids;
  const auto lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
  for (const auto & previous_lanelet : LaneletWrapper::routingGraph(type)->previous(lanelet)) {
    previous_lanelet_ids.push_back(previous_lanelet.id());
  }
  return previous_lanelet_ids;
}

auto previousLaneletIds(const lanelet::Ids & lanelet_ids, const RoutingGraphType type)
  -> lanelet::Ids
{
  std::set<lanelet::Id> previous_lanelet_ids_set;
  for (const auto & lanelet_id : lanelet_ids) {
    auto previous_lanelet_ids = previousLaneletIds(lanelet_id, type);
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
auto toPolygon(const lanelet::ConstLineString3d & line_string) -> std::vector<Point>
{
  std::vector<Point> points;
  points.reserve(line_string.size());
  for (const auto & point : line_string) {
    points.push_back(geometry_msgs::build<Point>().x(point.x()).y(point.y()).z(point.z()));
  }
  return points;
}
}  // namespace lanelet_map
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
