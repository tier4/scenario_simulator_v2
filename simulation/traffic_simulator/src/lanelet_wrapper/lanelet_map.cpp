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

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/normalize.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>
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

auto laneletYaw(const lanelet::Id lanelet_id, const Point & point)
  -> std::tuple<double, Point, Point>
{
  if (const auto centerline_points = lanelet_wrapper::lanelet_map::centerPoints(lanelet_id);
      centerline_points.empty()) {
    THROW_SIMULATION_ERROR(
      "There is no center points for lanelet with id: " + std::to_string(lanelet_id));
  } else {
    auto findNearestPointIndex = [](const std::vector<Point> & points, const Point & point) {
      return std::distance(
        points.begin(),
        std::min_element(points.begin(), points.end(), [&](const Point & p1, const Point & p2) {
          return math::geometry::hypot(p1, point) < math::geometry::hypot(p2, point);
        }));
    };
    const size_t nearest_point_index = findNearestPointIndex(centerline_points, point);
    const auto & nearest_point = centerline_points.at(nearest_point_index);
    const auto & next_point = centerline_points.at(nearest_point_index + 1);
    const auto yaw = std::atan2(next_point.y - nearest_point.y, next_point.x - nearest_point.x);
    return std::make_tuple(yaw, nearest_point, next_point);
  }
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
  auto excludeSubtypeLanelets =
    [](
      const std::vector<std::pair<double, lanelet::Lanelet>> & pair_distance_lanelet,
      const char subtype[]) -> std::vector<std::pair<double, lanelet::Lanelet>> {
    std::vector<std::pair<double, lanelet::Lanelet>> filtered_lanelets;
    for (const auto & pair : pair_distance_lanelet) {
      if (pair.second.hasAttribute(lanelet::AttributeName::Subtype)) {
        if (pair.second.attribute(lanelet::AttributeName::Subtype).value() != subtype) {
          filtered_lanelets.push_back(pair);
        }
      }
    }
    return filtered_lanelets;
  };

  auto nearest_lanelets = lanelet::geometry::findNearest(
    LaneletWrapper::map()->laneletLayer, lanelet::BasicPoint2d(point.x, point.y), search_count);

  // check for current content, if not empty then optionally apply filter
  if (nearest_lanelets.empty() || nearest_lanelets.front().first > distance_thresh) {
    return {};
  } else if (!include_crosswalk) {
    nearest_lanelets =
      excludeSubtypeLanelets(nearest_lanelets, lanelet::AttributeValueString::Crosswalk);
  }

  // check again
  if (nearest_lanelets.empty() || nearest_lanelets.front().first > distance_thresh) {
    return {};
  } else {
    lanelet::Ids target_lanelet_ids;
    for (const auto & lanelet : nearest_lanelets) {
      if (lanelet.first <= distance_thresh) {
        target_lanelet_ids.emplace_back(lanelet.second.id());
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

// Next lanelet
auto nextLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  auto nextRoadShoulderLaneletIds = [](const lanelet::Id reference_lanelet_id) -> lanelet::Ids {
    lanelet::Ids shoulder_lanelet_ids;
    const auto & reference_lanelet = LaneletWrapper::map()->laneletLayer.get(reference_lanelet_id);
    for (const auto & shoulder_lanelet : LaneletWrapper::shoulderLanelets()) {
      if (lanelet::geometry::follows(reference_lanelet, shoulder_lanelet)) {
        shoulder_lanelet_ids.push_back(shoulder_lanelet.id());
      }
    }
    return shoulder_lanelet_ids;
  };

  lanelet::Ids next_lanelet_ids;
  const auto & lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
  for (const auto & following_lanelet : LaneletWrapper::vehicleRoutingGraph()->following(lanelet)) {
    next_lanelet_ids.push_back(following_lanelet.id());
  }
  for (const auto & shoulder_lanelet_id : nextRoadShoulderLaneletIds(lanelet_id)) {
    next_lanelet_ids.push_back(shoulder_lanelet_id);
  }
  return next_lanelet_ids;
}

auto nextLaneletIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  std::set<lanelet::Id> next_lanelet_ids_set;
  for (const auto & lanelet_id : lanelet_ids) {
    auto next_lanelet_ids = nextLaneletIds(lanelet_id);
    next_lanelet_ids_set.insert(next_lanelet_ids.begin(), next_lanelet_ids.end());
  }
  return lanelet::Ids(next_lanelet_ids_set.begin(), next_lanelet_ids_set.end());
}

auto nextLaneletIds(const lanelet::Id lanelet_id, const std::string & turn_direction)
  -> lanelet::Ids
{
  lanelet::Ids next_lanelet_ids;
  const auto & reference_lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
  for (const auto & following_lanelet :
       LaneletWrapper::vehicleRoutingGraph()->following(reference_lanelet)) {
    if (following_lanelet.attributeOr("turn_direction", "else") == turn_direction) {
      next_lanelet_ids.push_back(following_lanelet.id());
    }
  }
  return next_lanelet_ids;
}

auto nextLaneletIds(const lanelet::Ids & lanelet_ids, const std::string & turn_direction)
  -> lanelet::Ids
{
  std::set<lanelet::Id> next_lanelet_ids_set;
  for (const auto & lanelet_id : lanelet_ids) {
    auto next_lanelet_ids = nextLaneletIds(lanelet_id, turn_direction);
    next_lanelet_ids_set.insert(next_lanelet_ids.begin(), next_lanelet_ids.end());
  }
  return lanelet::Ids(next_lanelet_ids_set.begin(), next_lanelet_ids_set.end());
}

//Previus lanelet
auto previousLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  auto previousRoadShoulderLanelet = [](const lanelet::Id reference_lanelet_id) -> lanelet::Ids {
    lanelet::Ids shoulder_lanelet_ids;
    const auto & reference_lanelet = LaneletWrapper::map()->laneletLayer.get(reference_lanelet_id);
    for (const auto & shoulder_lanelet : LaneletWrapper::shoulderLanelets()) {
      if (lanelet::geometry::follows(shoulder_lanelet, reference_lanelet)) {
        shoulder_lanelet_ids.push_back(shoulder_lanelet.id());
      }
    }
    return shoulder_lanelet_ids;
  };

  lanelet::Ids previous_lanelet_ids;
  const auto & lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
  for (const auto & previous_lanelet : LaneletWrapper::vehicleRoutingGraph()->previous(lanelet)) {
    previous_lanelet_ids.push_back(previous_lanelet.id());
  }
  for (const auto & shoulder_lanelet_id : previousRoadShoulderLanelet(lanelet_id)) {
    previous_lanelet_ids.push_back(shoulder_lanelet_id);
  }
  return previous_lanelet_ids;
}

auto previousLaneletIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  std::set<lanelet::Id> previous_lanelet_ids_set;
  for (const auto & lanelet_id : lanelet_ids) {
    auto previous_lanelet_ids = previousLaneletIds(lanelet_id);
    previous_lanelet_ids_set.insert(previous_lanelet_ids.begin(), previous_lanelet_ids.end());
  }
  return lanelet::Ids(previous_lanelet_ids_set.begin(), previous_lanelet_ids_set.end());
}

auto previousLaneletIds(const lanelet::Id lanelet_id, const std::string & turn_direction)
  -> lanelet::Ids
{
  lanelet::Ids previous_lanelet_ids;
  const auto reference_lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
  for (const auto & previous_lanelet :
       LaneletWrapper::vehicleRoutingGraph()->previous(reference_lanelet)) {
    if (previous_lanelet.attributeOr("turn_direction", "else") == turn_direction) {
      previous_lanelet_ids.push_back(previous_lanelet.id());
    }
  }
  return previous_lanelet_ids;
}

auto previousLaneletIds(const lanelet::Ids & lanelet_ids, const std::string & turn_direction)
  -> lanelet::Ids
{
  std::set<lanelet::Id> previous_lanelet_ids_set;
  for (const auto & lanelet_id : lanelet_ids) {
    auto previous_lanelet_ids = previousLaneletIds(lanelet_id, turn_direction);
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
  for (const auto & point : lanelet_polygon) {
    points.emplace_back(geometry_msgs::build<Point>().x(point.x()).y(point.y()).z(point.z()));
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
  for (const auto & point : line_string) {
    points.emplace_back(geometry_msgs::build<Point>().x(point.x()).y(point.y()).z(point.z()));
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
    right_of_way_lanelets_ids.emplace(lanelet_id, rightOfWayLaneletIds(lanelet_id));
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
    {LaneletWrapper::vehicleRoutingGraph(), LaneletWrapper::pedestrianRoutingGraph()});
  for (const auto & lanelet_id : lanelet_ids) {
    const auto & conflicting_crosswalks = graphs_container.conflictingInGraph(
      LaneletWrapper::map()->laneletLayer.get(lanelet_id), routing_graph_id, height_clearance);
    for (const auto & conflicting_crosswalk : conflicting_crosswalks) {
      conflicting_crosswalk_ids.emplace_back(conflicting_crosswalk.id());
    }
  }
  return conflicting_crosswalk_ids;
}

auto conflictingLaneIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids conflicting_lanes_ids;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto & conflicting_lanelets = lanelet::utils::getConflictingLanelets(
      LaneletWrapper::vehicleRoutingGraph(), LaneletWrapper::map()->laneletLayer.get(lanelet_id));
    for (const auto & conflicting_lanelet : conflicting_lanelets) {
      conflicting_lanes_ids.emplace_back(conflicting_lanelet.id());
    }
  }
  return conflicting_lanes_ids;
}
}  // namespace lanelet_map
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
