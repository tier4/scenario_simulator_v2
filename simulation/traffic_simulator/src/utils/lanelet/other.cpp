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

#include <geometry/linear_algebra.hpp>
#include <geometry/vector3/normalize.hpp>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/lanelet/memory.hpp>
#include <traffic_simulator/utils/lanelet/other.hpp>
#include <traffic_simulator/utils/lanelet/pose.hpp>

namespace traffic_simulator
{
namespace lanelet2
{
namespace other
{

auto getLaneletIds() -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & lanelet : Memory::laneletMap()->laneletLayer) {
    ids.push_back(lanelet.id());
  }
  return ids;
}

auto getLaneletLength(const lanelet::Id lanelet_id) -> double
{
  if (Memory::laneletLengthCache().exists(lanelet_id)) {
    return Memory::laneletLengthCache().getLength(lanelet_id);
  }
  double ret =
    lanelet::utils::getLaneletLength2d(Memory::laneletMap()->laneletLayer.get(lanelet_id));
  Memory::laneletLengthCache().appendData(lanelet_id, ret);
  return ret;
}

auto getCenterPoints(const lanelet::Ids & lanelet_ids) -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> ret;
  if (lanelet_ids.empty()) {
    return ret;
  }
  for (const auto lanelet_id : lanelet_ids) {
    ret += getCenterPoints(lanelet_id);
  }
  ret.erase(std::unique(ret.begin(), ret.end()), ret.end());
  return ret;
}

auto getCenterPoints(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> ret;
  if (!Memory::laneletMap()) {
    THROW_SIMULATION_ERROR("lanelet map is null pointer");
  }
  if (Memory::laneletMap()->laneletLayer.empty()) {
    THROW_SIMULATION_ERROR("lanelet layer is empty");
  }
  if (Memory::centerPointsCache().exists(lanelet_id)) {
    return Memory::centerPointsCache().getCenterPoints(lanelet_id);
  }

  const auto lanelet = Memory::laneletMap()->laneletLayer.get(lanelet_id);
  const auto centerline = lanelet.centerline();
  for (const auto & point : centerline) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    ret.push_back(p);
  }
  if (static_cast<int>(ret.size()) == 2) {
    const auto p0 = ret[0];
    const auto p2 = ret[1];
    geometry_msgs::msg::Point p1;
    p1.x = (p0.x + p2.x) * 0.5;
    p1.y = (p0.y + p2.y) * 0.5;
    p1.z = (p0.z + p2.z) * 0.5;
    ret.clear();
    ret.push_back(p0);
    ret.push_back(p1);
    ret.push_back(p2);
  }
  Memory::centerPointsCache().appendData(lanelet_id, ret);
  return ret;
}

auto getCenterPointsSpline(const lanelet::Id lanelet_id)
  -> std::shared_ptr<math::geometry::CatmullRomSpline>
{
  getCenterPoints(lanelet_id);
  return Memory::centerPointsCache().getCenterPointsSpline(lanelet_id);
}

auto getNextRoadShoulderLanelet(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = Memory::laneletMap()->laneletLayer.get(lanelet_id);
  for (const auto & shoulder_lanelet : Memory::shoulderLanelets()) {
    if (lanelet::geometry::follows(lanelet, shoulder_lanelet)) {
      ids.push_back(shoulder_lanelet.id());
    }
  }
  return ids;
}

auto getNextLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = Memory::laneletMap()->laneletLayer.get(lanelet_id);
  for (const auto & llt : Memory::vehicleRoutingGraph()->following(lanelet)) {
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
  const auto lanelet = Memory::laneletMap()->laneletLayer.get(lanelet_id);
  for (const auto & llt : Memory::vehicleRoutingGraph()->following(lanelet)) {
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

auto getPreviousRoadShoulderLanelet(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = Memory::laneletMap()->laneletLayer.get(lanelet_id);
  for (const auto & shoulder_lanelet : Memory::shoulderLanelets()) {
    if (lanelet::geometry::follows(shoulder_lanelet, lanelet)) {
      ids.push_back(shoulder_lanelet.id());
    }
  }
  return ids;
}

auto getPreviousLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto lanelet = Memory::laneletMap()->laneletLayer.get(lanelet_id);
  for (const auto & llt : Memory::vehicleRoutingGraph()->previous(lanelet)) {
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
  const auto lanelet = Memory::laneletMap()->laneletLayer.get(lanelet_id);
  for (const auto & llt : Memory::vehicleRoutingGraph()->previous(lanelet)) {
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
    ids += other::getPreviousLaneletIds(id, turn_direction);
  }
  return sortAndUnique(ids);
}

auto toMapPose(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose, const bool fill_pitch)
  -> geometry_msgs::msg::PoseStamped
{
  if (
    const auto pose = std::get<std::optional<traffic_simulator_msgs::msg::LaneletPose>>(
      pose::canonicalizeLaneletPose(lanelet_pose))) {
    geometry_msgs::msg::PoseStamped ret;
    ret.header.frame_id = "map";
    const auto spline = getCenterPointsSpline(pose->lanelet_id);
    ret.pose = spline->getPose(pose->s);
    const auto normal_vec = spline->getNormalVector(pose->s);
    const auto diff = math::geometry::normalize(normal_vec) * pose->offset;
    ret.pose.position = ret.pose.position + diff;
    const auto tangent_vec = spline->getTangentVector(pose->s);
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 0.0;
    rpy.y = fill_pitch ? std::atan2(-tangent_vec.z, std::hypot(tangent_vec.x, tangent_vec.y)) : 0.0;
    rpy.z = std::atan2(tangent_vec.y, tangent_vec.x);
    ret.pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy) *
                           quaternion_operation::convertEulerAngleToQuaternion(pose->rpy);
    return ret;
  } else {
    THROW_SEMANTIC_ERROR(
      "Lanelet pose (id=", lanelet_pose.lanelet_id, ",s=", lanelet_pose.s,
      ",offset=", lanelet_pose.offset, ",rpy.x=", lanelet_pose.rpy.x, ",rpy.y=", lanelet_pose.rpy.y,
      ",rpy.z=", lanelet_pose.rpy.z, ") is invalid, please check lanelet length and connection.");
  }
}

auto toPolygon(const lanelet::ConstLineString3d & line_string)
  -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> ret;
  for (const auto & p : line_string) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    ret.emplace_back(point);
  }
  return ret;
}

auto isInLanelet(const lanelet::Id lanelet_id, const double s) -> bool
{
  return 0 <= s and s <= getCenterPointsSpline(lanelet_id)->getLength();
}

auto getLaneletPolygon(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> points;
  lanelet::CompoundPolygon3d lanelet_polygon =
    Memory::laneletMap()->laneletLayer.get(lanelet_id).polygon3d();
  for (const auto & lanelet_point : lanelet_polygon) {
    geometry_msgs::msg::Point p;
    p.x = lanelet_point.x();
    p.y = lanelet_point.y();
    p.z = lanelet_point.z();
    points.emplace_back(p);
  }
  return points;
}

auto getStopLinePolygon(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> points;
  const auto stop_line = Memory::laneletMap()->lineStringLayer.get(lanelet_id);
  for (const auto & point : stop_line) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    points.emplace_back(p);
  }
  return points;
}

auto generateMarker() -> visualization_msgs::msg::MarkerArray
{
  visualization_msgs::msg::MarkerArray markers;
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(Memory::laneletMap());
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet::ConstLanelets crosswalk_lanelets =
    lanelet::utils::query::crosswalkLanelets(all_lanelets);
  lanelet::ConstLanelets walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);
  lanelet::ConstLineStrings3d stop_lines = lanelet::utils::query::stopLinesLanelets(road_lanelets);
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems =
    lanelet::utils::query::detectionAreas(all_lanelets);
  lanelet::ConstLineStrings3d parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(Memory::laneletMap());
  lanelet::ConstPolygons3d parking_lots =
    lanelet::utils::query::getAllParkingLots(Memory::laneletMap());

  auto cl_ll_borders = color_utils::fromRgba(1.0, 1.0, 1.0, 0.999);
  auto cl_road = color_utils::fromRgba(0.2, 0.7, 0.7, 0.3);
  auto cl_cross = color_utils::fromRgba(0.2, 0.7, 0.2, 0.3);
  auto cl_stoplines = color_utils::fromRgba(1.0, 0.0, 0.0, 0.5);
  auto cl_trafficlights = color_utils::fromRgba(0.7, 0.7, 0.7, 0.8);
  auto cl_detection_areas = color_utils::fromRgba(0.7, 0.7, 0.7, 0.3);
  auto cl_parking_lots = color_utils::fromRgba(0.7, 0.7, 0.0, 0.3);
  auto cl_parking_spaces = color_utils::fromRgba(1.0, 0.647, 0.0, 0.6);
  auto cl_lanelet_id = color_utils::fromRgba(0.8, 0.2, 0.2, 0.999);

  insertMarkerArray(
    markers,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(road_lanelets, cl_ll_borders, true));
  insertMarkerArray(
    markers,
    lanelet::visualization::laneletsAsTriangleMarkerArray("road_lanelets", road_lanelets, cl_road));
  insertMarkerArray(
    markers, lanelet::visualization::laneletsAsTriangleMarkerArray(
               "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
  insertMarkerArray(
    markers, lanelet::visualization::laneletsAsTriangleMarkerArray(
               "walkway_lanelets", walkway_lanelets, cl_cross));
  insertMarkerArray(markers, lanelet::visualization::laneletDirectionAsMarkerArray(road_lanelets));
  insertMarkerArray(
    markers,
    lanelet::visualization::lineStringsAsMarkerArray(stop_lines, "stop_lines", cl_stoplines, 0.1));
  insertMarkerArray(
    markers,
    lanelet::visualization::autowareTrafficLightsAsMarkerArray(aw_tl_reg_elems, cl_trafficlights));
  insertMarkerArray(
    markers, lanelet::visualization::detectionAreasAsMarkerArray(da_reg_elems, cl_detection_areas));
  insertMarkerArray(
    markers, lanelet::visualization::parkingLotsAsMarkerArray(parking_lots, cl_parking_lots));
  insertMarkerArray(
    markers, lanelet::visualization::parkingSpacesAsMarkerArray(parking_spaces, cl_parking_spaces));
  insertMarkerArray(
    markers, lanelet::visualization::generateLaneletIdMarker(road_lanelets, cl_lanelet_id));
  insertMarkerArray(
    markers, lanelet::visualization::generateLaneletIdMarker(crosswalk_lanelets, cl_lanelet_id));
  return markers;
}

auto insertMarkerArray(
  visualization_msgs::msg::MarkerArray & a1, const visualization_msgs::msg::MarkerArray & a2)
  -> void
{
  a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}
}  // namespace other
}  // namespace lanelet2
}  // namespace traffic_simulator
