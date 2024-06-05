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

#include <geometry/linear_algebra.hpp>
#include <geometry/vector3/normalize.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/lanelet_core/lanelet_map.hpp>
#include <traffic_simulator/utils/lanelet_core/lanelet_map_core.hpp>
#include <traffic_simulator/utils/lanelet_core/pose.hpp>

namespace traffic_simulator
{
namespace lanelet_core
{
namespace lanelet_map
{
auto isInLanelet(const double s, const lanelet::Id lanelet_id) -> bool
{
  return 0 <= s and s <= getCenterPointsSpline(lanelet_id)->getLength();
}

auto isInLanelet(const Point point, const lanelet::Id lanelet_id) -> bool
{
  return lanelet::geometry::inside(
    LaneletMapCore::map()->laneletLayer.get(lanelet_id), lanelet::BasicPoint2d(point.x, point.y));
}

auto getLaneletLength(const lanelet::Id lanelet_id) -> double
{
  if (LaneletMapCore::laneletLengthCache().exists(lanelet_id)) {
    return LaneletMapCore::laneletLengthCache().getLength(lanelet_id);
  }
  double ret =
    lanelet::utils::getLaneletLength2d(LaneletMapCore::map()->laneletLayer.get(lanelet_id));
  LaneletMapCore::laneletLengthCache().appendData(lanelet_id, ret);
  return ret;
}

auto getLaneletIds() -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & lanelet : LaneletMapCore::map()->laneletLayer) {
    ids.push_back(lanelet.id());
  }
  return ids;
}

auto getNearbyLaneletIds(
  const Point & point, const double distance_thresh, const bool include_crosswalk,
  const std::size_t search_count) -> lanelet::Ids
{
  lanelet::Ids lanelet_ids;
  lanelet::BasicPoint2d search_point(point.x, point.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet =
    lanelet::geometry::findNearest(LaneletMapCore::map()->laneletLayer, search_point, search_count);
  if (include_crosswalk) {
    if (nearest_lanelet.empty()) {
      return {};
    }
    if (nearest_lanelet.front().first > distance_thresh) {
      return {};
    }
    for (const auto & lanelet : nearest_lanelet) {
      if (lanelet.first <= distance_thresh) {
        lanelet_ids.emplace_back(lanelet.second.id());
      }
    }
  } else {
    const auto nearest_road_lanelet =
      excludeSubtypeLanelets(nearest_lanelet, lanelet::AttributeValueString::Crosswalk);
    if (nearest_road_lanelet.empty()) {
      return {};
    }
    if (nearest_road_lanelet.front().first > distance_thresh) {
      return {};
    }
    for (const auto & lanelet : nearest_road_lanelet) {
      if (lanelet.first <= distance_thresh) {
        lanelet_ids.emplace_back(lanelet.second.id());
      }
    }
  }
  return lanelet_ids;
}

// Center points
auto getCenterPoints(const lanelet::Ids & lanelet_ids) -> std::vector<Point>
{
  std::vector<Point> ret;
  if (lanelet_ids.empty()) {
    return ret;
  }
  for (const auto lanelet_id : lanelet_ids) {
    ret += getCenterPoints(lanelet_id);
  }
  ret.erase(std::unique(ret.begin(), ret.end()), ret.end());
  return ret;
}

auto getCenterPoints(const lanelet::Id lanelet_id) -> std::vector<Point>
{
  std::vector<Point> ret;
  if (!LaneletMapCore::map()) {
    THROW_SIMULATION_ERROR("lanelet map is null pointer");
  }
  if (LaneletMapCore::map()->laneletLayer.empty()) {
    THROW_SIMULATION_ERROR("lanelet layer is empty");
  }
  if (LaneletMapCore::centerPointsCache().exists(lanelet_id)) {
    return LaneletMapCore::centerPointsCache().getCenterPoints(lanelet_id);
  }

  const auto lanelet = LaneletMapCore::map()->laneletLayer.get(lanelet_id);
  const auto centerline = lanelet.centerline();
  for (const auto & point : centerline) {
    Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    ret.push_back(p);
  }
  if (static_cast<int>(ret.size()) == 2) {
    const auto p0 = ret[0];
    const auto p2 = ret[1];
    Point p1;
    p1.x = (p0.x + p2.x) * 0.5;
    p1.y = (p0.y + p2.y) * 0.5;
    p1.z = (p0.z + p2.z) * 0.5;
    ret.clear();
    ret.push_back(p0);
    ret.push_back(p1);
    ret.push_back(p2);
  }
  LaneletMapCore::centerPointsCache().appendData(lanelet_id, ret);
  return ret;
}

auto getCenterPointsSpline(const lanelet::Id lanelet_id) -> std::shared_ptr<Spline>
{
  getCenterPoints(lanelet_id);
  return LaneletMapCore::centerPointsCache().getCenterPointsSpline(lanelet_id);
}

// Next lanelet
auto getNextLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  auto getNextRoadShoulderLanelet = [](const lanelet::Id lanelet_id) -> lanelet::Ids {
    lanelet::Ids ids;
    const auto lanelet = LaneletMapCore::map()->laneletLayer.get(lanelet_id);
    for (const auto & shoulder_lanelet : LaneletMapCore::shoulderLanelets()) {
      if (lanelet::geometry::follows(lanelet, shoulder_lanelet)) {
        ids.push_back(shoulder_lanelet.id());
      }
    }
    return ids;
  };

  lanelet::Ids ids;
  const auto lanelet = LaneletMapCore::map()->laneletLayer.get(lanelet_id);
  for (const auto & llt : LaneletMapCore::vehicleRoutingGraph()->following(lanelet)) {
    ids.push_back(llt.id());
  }
  for (const auto & id : getNextRoadShoulderLanelet(lanelet_id)) {
    ids.push_back(id);
  }
  return ids;
}

auto getNextLaneletIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & id : lanelet_ids) {
    ids += getNextLaneletIds(id);
  }
  return sortAndUnique(ids);
}

auto getNextLaneletIds(const lanelet::Id lanelet_id, const std::string & turn_direction)
  -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = LaneletMapCore::map()->laneletLayer.get(lanelet_id);
  for (const auto & llt : LaneletMapCore::vehicleRoutingGraph()->following(lanelet)) {
    if (llt.attributeOr("turn_direction", "else") == turn_direction) {
      ids.push_back(llt.id());
    }
  }
  return ids;
}

auto getNextLaneletIds(const lanelet::Ids & lanelet_ids, const std::string & turn_direction)
  -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & id : lanelet_ids) {
    ids += getNextLaneletIds(id, turn_direction);
  }
  return sortAndUnique(ids);
}

//Previus lanelet
auto getPreviousLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  auto getPreviousRoadShoulderLanelet = [](const lanelet::Id lanelet_id) -> lanelet::Ids {
    lanelet::Ids ids;
    const auto lanelet = LaneletMapCore::map()->laneletLayer.get(lanelet_id);
    for (const auto & shoulder_lanelet : LaneletMapCore::shoulderLanelets()) {
      if (lanelet::geometry::follows(shoulder_lanelet, lanelet)) {
        ids.push_back(shoulder_lanelet.id());
      }
    }
    return ids;
  };

  lanelet::Ids ids;
  const auto lanelet = LaneletMapCore::map()->laneletLayer.get(lanelet_id);
  for (const auto & llt : LaneletMapCore::vehicleRoutingGraph()->previous(lanelet)) {
    ids.push_back(llt.id());
  }
  for (const auto & id : getPreviousRoadShoulderLanelet(lanelet_id)) {
    ids.push_back(id);
  }
  return ids;
}

auto getPreviousLaneletIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & id : lanelet_ids) {
    ids += getNextLaneletIds(id);
  }
  return sortAndUnique(ids);
}

auto getPreviousLaneletIds(const lanelet::Id lanelet_id, const std::string & turn_direction)
  -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = LaneletMapCore::map()->laneletLayer.get(lanelet_id);
  for (const auto & llt : LaneletMapCore::vehicleRoutingGraph()->previous(lanelet)) {
    if (llt.attributeOr("turn_direction", "else") == turn_direction) {
      ids.push_back(llt.id());
    }
  }
  return ids;
}

auto getPreviousLaneletIds(const lanelet::Ids & lanelet_ids, const std::string & turn_direction)
  -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & id : lanelet_ids) {
    ids += getPreviousLaneletIds(id, turn_direction);
  }
  return sortAndUnique(ids);
}

// Bounds
auto getLeftBound(const lanelet::Id lanelet_id) -> std::vector<Point>
{
  return toPolygon(LaneletMapCore::map()->laneletLayer.get(lanelet_id).leftBound());
}

auto getRightBound(const lanelet::Id lanelet_id) -> std::vector<Point>
{
  return toPolygon(LaneletMapCore::map()->laneletLayer.get(lanelet_id).rightBound());
}

// Polygons
auto getLaneletPolygon(const lanelet::Id lanelet_id) -> std::vector<Point>
{
  std::vector<Point> points;
  lanelet::CompoundPolygon3d lanelet_polygon =
    LaneletMapCore::map()->laneletLayer.get(lanelet_id).polygon3d();
  for (const auto & lanelet_point : lanelet_polygon) {
    Point p;
    p.x = lanelet_point.x();
    p.y = lanelet_point.y();
    p.z = lanelet_point.z();
    points.emplace_back(p);
  }
  return points;
}

auto getStopLinePolygon(const lanelet::Id lanelet_id) -> std::vector<Point>
{
  std::vector<Point> points;
  const auto stop_line = LaneletMapCore::map()->lineStringLayer.get(lanelet_id);
  for (const auto & point : stop_line) {
    Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    points.emplace_back(p);
  }
  return points;
}

auto getRightOfWayLaneletIds(const lanelet::Ids & lanelet_ids)
  -> std::unordered_map<lanelet::Id, lanelet::Ids>
{
  std::unordered_map<lanelet::Id, lanelet::Ids> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    ret.emplace(lanelet_id, getRightOfWayLaneletIds(lanelet_id));
  }
  return ret;
}

// Relations
auto getRightOfWayLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & right_of_way : LaneletMapCore::map()
                                     ->laneletLayer.get(lanelet_id)
                                     .regulatoryElementsAs<lanelet::RightOfWay>()) {
    for (const auto & ll : right_of_way->rightOfWayLanelets()) {
      if (lanelet_id != ll.id()) {
        ids.push_back(ll.id());
      }
    }
  }
  return ids;
}

auto getConflictingCrosswalkIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids ids;
  std::vector<lanelet::routing::RoutingGraphConstPtr> graphs;
  graphs.emplace_back(LaneletMapCore::vehicleRoutingGraph());
  graphs.emplace_back(LaneletMapCore::pedestrianRoutingGraph());
  lanelet::routing::RoutingGraphContainer container(graphs);
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = LaneletMapCore::map()->laneletLayer.get(lanelet_id);
    double height_clearance = 4;
    size_t routing_graph_id = 1;
    const auto conflicting_crosswalks =
      container.conflictingInGraph(lanelet, routing_graph_id, height_clearance);
    for (const auto & crosswalk : conflicting_crosswalks) {
      ids.emplace_back(crosswalk.id());
    }
  }
  return ids;
}

auto getConflictingLaneIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = LaneletMapCore::map()->laneletLayer.get(lanelet_id);
    const auto conflicting_lanelets =
      lanelet::utils::getConflictingLanelets(LaneletMapCore::vehicleRoutingGraph(), lanelet);
    for (const auto & conflicting_lanelet : conflicting_lanelets) {
      ids.emplace_back(conflicting_lanelet.id());
    }
  }
  return ids;
}

// private
namespace
{
auto toPolygon(const lanelet::ConstLineString3d & line_string) -> std::vector<Point>
{
  std::vector<Point> ret;
  for (const auto & p : line_string) {
    Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    ret.emplace_back(point);
  }
  return ret;
}

auto excludeSubtypeLanelets(
  const std::vector<std::pair<double, lanelet::Lanelet>> & lls, const char subtype[])
  -> std::vector<std::pair<double, lanelet::Lanelet>>
{
  std::vector<std::pair<double, lanelet::Lanelet>> exclude_subtype_lanelets;
  for (const auto & ll : lls) {
    if (ll.second.hasAttribute(lanelet::AttributeName::Subtype)) {
      lanelet::Attribute attr = ll.second.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() != subtype) {
        exclude_subtype_lanelets.push_back(ll);
      }
    }
  }
  return exclude_subtype_lanelets;
}
}  // namespace
}  // namespace lanelet_map
}  // namespace lanelet_core
}  // namespace traffic_simulator
