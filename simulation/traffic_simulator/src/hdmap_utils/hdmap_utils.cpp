// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <lanelet2_core/utility/Units.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_projection/UTM.h>
#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <deque>
#include <lanelet2_extension_psim/io/autoware_osm_parser.hpp>
#include <lanelet2_extension_psim/projection/mgrs_projector.hpp>
#include <lanelet2_extension_psim/utility/message_conversion.hpp>
#include <lanelet2_extension_psim/utility/query.hpp>
#include <lanelet2_extension_psim/utility/utilities.hpp>
#include <lanelet2_extension_psim/visualization/visualization.hpp>
#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
#include <traffic_simulator/math/hermite_curve.hpp>
#include <traffic_simulator/math/linear_algebra.hpp>
#include <traffic_simulator/math/transfrom.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hdmap_utils
{
HdMapUtils::HdMapUtils(
  const boost::filesystem::path & lanelet2_map_path, const geographic_msgs::msg::GeoPoint & origin)
{
  (void)origin;

  lanelet::projection::MGRSProjector projector;

  lanelet::ErrorMessages errors;

  lanelet_map_ptr_ = lanelet::load(lanelet2_map_path.string(), projector, &errors);

  if (not errors.empty()) {
    std::stringstream ss;
    const auto * separator = "";
    for (const auto & error : errors) {
      ss << separator << error;
      separator = "\n";
    }
    THROW_SIMULATION_ERROR("Failed to load lanelet map (", ss.str(), ")");
  }
  overwriteLaneletsCenterline();
  traffic_rules_vehicle_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  vehicle_routing_graph_ptr_ =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules_vehicle_ptr_);
  traffic_rules_pedestrian_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  pedestrian_routing_graph_ptr_ =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules_pedestrian_ptr_);
  std::vector<lanelet::routing::RoutingGraphConstPtr> all_graphs;
  all_graphs.push_back(vehicle_routing_graph_ptr_);
  all_graphs.push_back(pedestrian_routing_graph_ptr_);
}

const std::vector<std::int64_t> HdMapUtils::getLaneletIds()
{
  std::vector<std::int64_t> ret;
  for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
    ret.emplace_back(lanelet.id());
  }
  return ret;
}

const std::vector<geometry_msgs::msg::Point> HdMapUtils::getLaneletPolygon(std::int64_t lanelet_id)
{
  std::vector<geometry_msgs::msg::Point> points;
  lanelet::CompoundPolygon3d lanelet_polygon =
    lanelet_map_ptr_->laneletLayer.get(lanelet_id).polygon3d();
  for (const auto & lanelet_point : lanelet_polygon) {
    geometry_msgs::msg::Point p;
    p.x = lanelet_point.x();
    p.y = lanelet_point.y();
    p.z = lanelet_point.z();
    points.emplace_back(p);
  }
  return points;
}

std::vector<std::int64_t> HdMapUtils::filterLaneletIds(
  const std::vector<std::int64_t> & lanelet_ids, const char subtype[]) const
{
  const auto lanelets = getLanelets(lanelet_ids);
  std::vector<lanelet::Lanelet> filtered_lanelets;
  for (const auto & ll : lanelets) {
    if (ll.hasAttribute(lanelet::AttributeName::Subtype)) {
      lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == subtype) {
        filtered_lanelets.emplace_back(ll);
      }
    }
  }
  return getLaneletIds(filtered_lanelets);
}

std::vector<std::int64_t> HdMapUtils::getNearbyLaneletIds(
  const geometry_msgs::msg::Point & position, double distance_threshold) const
{
  std::vector<std::int64_t> lanelet_ids;
  lanelet::BasicPoint2d search_point(position.x, position.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 5);
  if (nearest_lanelet.empty()) {
    return {};
  }
  for (const auto & lanelet : nearest_lanelet) {
    if (lanelet.first <= distance_threshold) {
      lanelet_ids.emplace_back(lanelet.second.id());
    }
  }
  return lanelet_ids;
}

std::vector<std::int64_t> HdMapUtils::getNearbyLaneletIds(
  const geometry_msgs::msg::Point & point, double distance_thresh, bool include_crosswalk) const
{
  std::vector<std::int64_t> lanelet_ids;
  lanelet::BasicPoint2d search_point(point.x, point.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 5);
  if (include_crosswalk) {
    if (nearest_lanelet.empty()) {
      return {};
    }
    if (nearest_lanelet.front().first > distance_thresh) {
      return {};
    }
    for (const auto & lanelet : nearest_lanelet) {
      lanelet_ids.emplace_back(lanelet.second.id());
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
    for (const auto & lanelet : nearest_lanelet) {
      lanelet_ids.emplace_back(lanelet.second.id());
    }
  }
  return lanelet_ids;
}

double HdMapUtils::getHeight(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
{
  return toMapPose(lanelet_pose).pose.position.z;
}

boost::optional<double> HdMapUtils::getCollisionPointInLaneCoordinate(
  std::int64_t lanelet_id, std::int64_t crossing_lanelet_id)
{
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Line = bg::model::linestring<Point>;
  using Polygon = bg::model::polygon<Point, false>;
  auto center_points = getCenterPoints(lanelet_id);
  std::vector<Point> path_collision_points;
  lanelet_map_ptr_->laneletLayer.get(crossing_lanelet_id);
  lanelet::CompoundPolygon3d lanelet_polygon =
    lanelet_map_ptr_->laneletLayer.get(crossing_lanelet_id).polygon3d();
  Polygon crosswalk_polygon;
  for (const auto & lanelet_point : lanelet_polygon) {
    crosswalk_polygon.outer().push_back(bg::make<Point>(lanelet_point.x(), lanelet_point.y()));
  }
  crosswalk_polygon.outer().push_back(crosswalk_polygon.outer().front());
  double s_in_lanelet = 0;
  for (size_t i = 0; i < center_points.size() - 1; ++i) {
    const auto p0 = center_points.at(i);
    const auto p1 = center_points.at(i + 1);
    const Line line{{p0.x, p0.y}, {p1.x, p1.y}};
    double line_length =
      std::sqrt(std::pow(p0.x - p1.x, 2) + std::pow(p0.y - p1.y, 2) + std::pow(p0.z - p1.z, 2));
    std::vector<Point> line_collision_points;
    bg::intersection(crosswalk_polygon, line, line_collision_points);
    if (line_collision_points.empty()) {
      continue;
    }
    std::vector<double> dist;
    for (size_t j = 0; j < line_collision_points.size(); ++j) {
      double s_in_line = 0;
      if (std::fabs(p1.x - p0.x) < DBL_EPSILON) {
        if (std::fabs(p1.y - p1.y < DBL_EPSILON)) {
        } else {
          s_in_line = (line_collision_points[j].y() - p0.y) / (p1.y - p0.y);
          return s_in_lanelet + s_in_line * line_length;
        }
      } else {
        s_in_line = (line_collision_points[j].x() - p0.x) / (p1.x - p0.x);
        return s_in_lanelet + s_in_line * line_length;
      }
    }
    s_in_lanelet = s_in_lanelet + line_length;
  }
  return boost::none;
}

std::vector<std::int64_t> HdMapUtils::getConflictingLaneIds(
  const std::vector<std::int64_t> & lanelet_ids) const
{
  std::vector<std::int64_t> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const auto conflicting_lanelets =
      lanelet::utils::getConflictingLanelets(vehicle_routing_graph_ptr_, lanelet);
    for (const auto & conflicting_lanelet : conflicting_lanelets) {
      ret.emplace_back(conflicting_lanelet.id());
    }
  }
  return ret;
}

std::vector<std::int64_t> HdMapUtils::getConflictingCrosswalkIds(
  const std::vector<std::int64_t> & lanelet_ids) const
{
  std::vector<std::int64_t> ret;
  std::vector<lanelet::routing::RoutingGraphConstPtr> graphs;
  graphs.emplace_back(vehicle_routing_graph_ptr_);
  graphs.emplace_back(pedestrian_routing_graph_ptr_);
  lanelet::routing::RoutingGraphContainer container(graphs);
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    double height_clearance = 4;
    size_t routing_graph_id = 1;
    const auto conflicting_crosswalks =
      container.conflictingInGraph(lanelet, routing_graph_id, height_clearance);
    for (const auto & crosswalk : conflicting_crosswalks) {
      ret.emplace_back(crosswalk.id());
    }
  }
  return ret;
}

std::vector<geometry_msgs::msg::Point> HdMapUtils::clipTrajectoryFromLaneletIds(
  std::int64_t lanelet_id, double s, std::vector<std::int64_t> lanelet_ids, double forward_distance)
{
  std::vector<geometry_msgs::msg::Point> ret;
  bool on_traj = false;
  double rest_distance = forward_distance;
  for (auto id_itr = lanelet_ids.begin(); id_itr != lanelet_ids.end(); id_itr++) {
    double l = getLaneletLength(*id_itr);
    if (on_traj) {
      if (rest_distance < l) {
        for (double s_val = 0; s_val < rest_distance; s_val = s_val + 1.0) {
          auto map_pose = toMapPose(*id_itr, s_val, 0);
          ret.emplace_back(map_pose.pose.position);
        }
        break;
      } else {
        rest_distance = rest_distance - l;
        for (double s_val = 0; s_val < l; s_val = s_val + 1.0) {
          auto map_pose = toMapPose(*id_itr, s_val, 0);
          ret.emplace_back(map_pose.pose.position);
        }
        continue;
      }
    }
    if (lanelet_id == *id_itr) {
      on_traj = true;
      if ((s + forward_distance) < l) {
        for (double s_val = s; s_val < s + forward_distance; s_val = s_val + 1.0) {
          auto map_pose = toMapPose(lanelet_id, s_val, 0);
          ret.emplace_back(map_pose.pose.position);
        }
        break;
      } else {
        rest_distance = rest_distance - (l - s);
        for (double s_val = s; s_val < l; s_val = s_val + 1.0) {
          auto map_pose = toMapPose(lanelet_id, s_val, 0);
          ret.emplace_back(map_pose.pose.position);
        }
        continue;
      }
    }
  }
  return ret;
}

std::vector<lanelet::Lanelet> HdMapUtils::filterLanelets(
  const std::vector<lanelet::Lanelet> & lanelets, const char subtype[]) const
{
  std::vector<lanelet::Lanelet> filtered_lanelets;
  for (const auto & ll : lanelets) {
    if (ll.hasAttribute(lanelet::AttributeName::Subtype)) {
      lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() != subtype) {
        filtered_lanelets.push_back(ll);
      }
    }
  }
  return filtered_lanelets;
}

std::vector<std::pair<double, lanelet::Lanelet>> HdMapUtils::excludeSubtypeLanelets(
  const std::vector<std::pair<double, lanelet::Lanelet>> & lls, const char subtype[]) const
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

lanelet::BasicPolygon2d HdMapUtils::absoluteHull(
  const lanelet::BasicPolygon2d & relativeHull, const lanelet::matching::Pose2d & pose) const
{
  lanelet::BasicPolygon2d hullPoints;
  hullPoints.reserve(relativeHull.size());
  for (const auto & hullPt : relativeHull) {
    hullPoints.push_back(pose * hullPt);
  }
  return hullPoints;
}

lanelet::BasicPoint2d HdMapUtils::toPoint2d(const geometry_msgs::msg::Point & point) const
{
  return lanelet::BasicPoint2d{point.x, point.y};
}

boost::optional<std::int64_t> HdMapUtils::matchToLane(
  const geometry_msgs::msg::Pose & pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
  bool include_crosswalk, double reduction_ratio) const
{
  boost::optional<std::int64_t> id;
  lanelet::matching::Object2d obj;
  obj.pose.translation() = toPoint2d(pose.position);
  obj.pose.linear() = Eigen::Rotation2D<double>(
                        quaternion_operation::convertQuaternionToEulerAngle(pose.orientation).z)
                        .matrix();
  obj.absoluteHull = absoluteHull(
    lanelet::matching::Hull2d{
      lanelet::BasicPoint2d{
        bbox.center.x + bbox.dimensions.x * 0.5 * reduction_ratio,
        bbox.center.y + bbox.dimensions.y * 0.5 * reduction_ratio},
      lanelet::BasicPoint2d{
        bbox.center.x - bbox.dimensions.x * 0.5 * reduction_ratio,
        bbox.center.y - bbox.dimensions.y * 0.5 * reduction_ratio}},
    obj.pose);
  auto matches = lanelet::matching::getDeterministicMatches(*lanelet_map_ptr_, obj, 3.0);
  if (!include_crosswalk) {
    matches = lanelet::matching::removeNonRuleCompliantMatches(matches, traffic_rules_vehicle_ptr_);
  }
  if (matches.empty()) {
    return boost::none;
  }
  std::sort(matches.begin(), matches.end(), [](auto const & lhs, auto const & rhs) {
    return lhs.distance < rhs.distance;
  });
  return matches[0].lanelet.id();
}

boost::optional<traffic_simulator_msgs::msg::LaneletPose> HdMapUtils::toLaneletPose(
  geometry_msgs::msg::Pose pose, bool include_crosswalk)
{
  const auto lanelet_ids = getNearbyLaneletIds(pose.position, 3.0, include_crosswalk);
  if (lanelet_ids.empty()) {
    return boost::none;
  }
  for (const auto & id : lanelet_ids) {
    const auto lanelet_pose = toLaneletPose(pose, id);
    if (lanelet_pose) {
      return lanelet_pose;
    }
  }
  return boost::none;
}

boost::optional<traffic_simulator_msgs::msg::LaneletPose> HdMapUtils::toLaneletPose(
  geometry_msgs::msg::Pose pose, std::int64_t lanelet_id)
{
  const auto spline = getCenterPointsSpline(lanelet_id);
  const auto s = spline->getSValue(pose);
  if (!s) {
    return boost::none;
  }
  auto pose_on_centerline = spline->getPose(s.get());
  auto rpy = quaternion_operation::convertQuaternionToEulerAngle(
    quaternion_operation::getRotation(pose_on_centerline.orientation, pose.orientation));
  double offset = spline->getSquaredDistanceIn2D(pose.position, s.get());
  traffic_simulator_msgs::msg::LaneletPose lanelet_pose;
  lanelet_pose.lanelet_id = lanelet_id;
  lanelet_pose.s = s.get();
  lanelet_pose.offset = offset;
  lanelet_pose.rpy = rpy;
  return lanelet_pose;
}

boost::optional<traffic_simulator_msgs::msg::LaneletPose> HdMapUtils::toLaneletPose(
  geometry_msgs::msg::Pose pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
  bool include_crosswalk)
{
  const auto lanelet_id = matchToLane(pose, bbox, include_crosswalk);
  if (!lanelet_id) {
    return toLaneletPose(pose, include_crosswalk);
  }
  const auto pose_in_target_lanelet = toLaneletPose(pose, lanelet_id.get());
  if (pose_in_target_lanelet) {
    return pose_in_target_lanelet;
  }
  const auto previous = getPreviousLaneletIds(lanelet_id.get());
  for (const auto id : previous) {
    const auto pose_in_previous = toLaneletPose(pose, id);
    if (pose_in_previous) {
      return pose_in_previous;
    }
  }
  const auto next = getNextLaneletIds(lanelet_id.get());
  for (const auto id : previous) {
    const auto pose_in_next = toLaneletPose(pose, id);
    if (pose_in_next) {
      return pose_in_next;
    }
  }
  return toLaneletPose(pose, include_crosswalk);
}

boost::optional<std::int64_t> HdMapUtils::getClosestLaneletId(
  geometry_msgs::msg::Pose pose, double distance_thresh, bool include_crosswalk)
{
  lanelet::BasicPoint2d search_point(pose.position.x, pose.position.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 3);
  if (include_crosswalk) {
    if (nearest_lanelet.empty()) {
      return boost::none;
    }
    if (nearest_lanelet.front().first > distance_thresh) {
      return boost::none;
    }
    lanelet::Lanelet closest_lanelet;
    closest_lanelet = nearest_lanelet.front().second;
    return closest_lanelet.id();
  } else {
    const auto nearest_road_lanelet =
      excludeSubtypeLanelets(nearest_lanelet, lanelet::AttributeValueString::Crosswalk);
    if (nearest_road_lanelet.empty()) {
      return boost::none;
    }
    if (nearest_road_lanelet.front().first > distance_thresh) {
      return boost::none;
    }
    lanelet::Lanelet closest_lanelet;
    closest_lanelet = nearest_road_lanelet.front().second;
    return closest_lanelet.id();
  }
}

double HdMapUtils::getSpeedLimit(std::vector<std::int64_t> lanelet_ids)
{
  std::vector<double> limits;
  if (lanelet_ids.empty()) {
    THROW_SEMANTIC_ERROR("size of the vector lanelet ids should be more than 1");
  }
  for (auto itr = lanelet_ids.begin(); itr != lanelet_ids.end(); itr++) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(*itr);
    const auto limit = traffic_rules_vehicle_ptr_->speedLimit(lanelet);
    limits.push_back(lanelet::units::KmHQuantity(limit.speedLimit).value() / 3.6);
  }
  return *std::min_element(limits.begin(), limits.end());
}

boost::optional<int64_t> HdMapUtils::getLaneChangeableLaneletId(
  std::int64_t lanelet_id, traffic_simulator::lane_change::Direction direction, uint8_t shift)
{
  if (shift == 0) {
    return getLaneChangeableLaneletId(
      lanelet_id, traffic_simulator::lane_change::Direction::STRAIGHT);
  } else {
    std::int64_t reference_id = lanelet_id;
    for (uint8_t i = 0; i < shift; i++) {
      auto id = getLaneChangeableLaneletId(reference_id, direction);
      if (!id) {
        return boost::none;
      } else {
        reference_id = id.get();
      }
      if (i == (shift - 1)) {
        return reference_id;
      }
    }
  }
  return boost::none;
}

boost::optional<std::int64_t> HdMapUtils::getLaneChangeableLaneletId(
  std::int64_t lanelet_id, traffic_simulator::lane_change::Direction direction)
{
  const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  boost::optional<std::int64_t> target = boost::none;
  switch (direction) {
    case traffic_simulator::lane_change::Direction::STRAIGHT:
      target = lanelet.id();
      break;
    case traffic_simulator::lane_change::Direction::LEFT:
      if (vehicle_routing_graph_ptr_->left(lanelet)) {
        target = vehicle_routing_graph_ptr_->left(lanelet)->id();
      }
      break;
    case traffic_simulator::lane_change::Direction::RIGHT:
      if (vehicle_routing_graph_ptr_->right(lanelet)) {
        target = vehicle_routing_graph_ptr_->right(lanelet)->id();
      }
      break;
  }
  return target;
}

std::vector<std::int64_t> HdMapUtils::getPreviousLanelets(std::int64_t lanelet_id, double distance)
{
  std::vector<std::int64_t> ret;
  double total_distance = 0.0;
  ret.push_back(lanelet_id);
  while (total_distance < distance) {
    auto ids = getPreviousLaneletIds(lanelet_id, "straight");
    if (ids.size() != 0) {
      lanelet_id = ids[0];
      total_distance = total_distance + getLaneletLength(lanelet_id);
      ret.push_back(lanelet_id);
      continue;
    } else {
      auto else_ids = getPreviousLaneletIds(lanelet_id);
      if (else_ids.size() != 0) {
        lanelet_id = else_ids[0];
        total_distance = total_distance + getLaneletLength(lanelet_id);
        ret.push_back(lanelet_id);
        continue;
      } else {
        break;
      }
    }
  }
  return ret;
}

bool HdMapUtils::isInRoute(std::int64_t lanelet_id, std::vector<std::int64_t> route) const
{
  for (const auto id : route) {
    if (id == lanelet_id) {
      return true;
    }
  }
  return false;
}

std::vector<std::int64_t> HdMapUtils::getFollowingLanelets(
  std::int64_t lanelet_id, std::vector<std::int64_t> candidate_lanelet_ids, double distance,
  bool include_self)
{
  if (candidate_lanelet_ids.empty()) {
    return {};
  }
  std::vector<std::int64_t> ret;
  double total_distance = 0.0;
  bool found = false;
  for (const auto id : candidate_lanelet_ids) {
    if (found) {
      ret.emplace_back(id);
      total_distance = total_distance + getLaneletLength(id);
      if (total_distance > distance) {
        return ret;
      }
    }
    if (id == lanelet_id) {
      found = true;
      if (include_self) {
        ret.emplace_back(id);
      }
    }
  }
  if (!found) {
    THROW_SEMANTIC_ERROR("lanelet id does not match");
  }
  if (total_distance > distance) {
    return ret;
  }
  std::int64_t end_lanelet = candidate_lanelet_ids[candidate_lanelet_ids.size() - 1];
  const auto followings = getFollowingLanelets(end_lanelet, distance - total_distance, false);
  std::copy(followings.begin(), followings.end(), std::back_inserter(ret));
  return ret;
}

std::vector<std::int64_t> HdMapUtils::getFollowingLanelets(
  std::int64_t lanelet_id, double distance, bool include_self)
{
  std::vector<std::int64_t> ret;
  double total_distance = 0.0;
  if (include_self) {
    ret.push_back(lanelet_id);
  }
  while (total_distance < distance) {
    const auto straight_ids = getNextLaneletIds(lanelet_id, "straight");
    if (straight_ids.size() != 0) {
      lanelet_id = straight_ids[0];
      total_distance = total_distance + getLaneletLength(lanelet_id);
      ret.push_back(lanelet_id);
      continue;
    }
    const auto ids = getNextLaneletIds(lanelet_id);
    if (ids.size() != 0) {
      lanelet_id = ids[0];
      total_distance = total_distance + getLaneletLength(lanelet_id);
      ret.push_back(lanelet_id);
      continue;
    } else {
      break;
    }
  }
  return ret;
}

std::vector<std::int64_t> HdMapUtils::getRoute(
  std::int64_t from_lanelet_id, std::int64_t to_lanelet_id)
{
  if (route_cache_.exists(from_lanelet_id, to_lanelet_id)) {
    return route_cache_.getRoute(from_lanelet_id, to_lanelet_id);
  }
  std::vector<std::int64_t> ret;
  const auto lanelet = lanelet_map_ptr_->laneletLayer.get(from_lanelet_id);
  const auto to_lanelet = lanelet_map_ptr_->laneletLayer.get(to_lanelet_id);
  lanelet::Optional<lanelet::routing::Route> route =
    vehicle_routing_graph_ptr_->getRoute(lanelet, to_lanelet, 0, false);
  if (!route) {
    route_cache_.appendData(from_lanelet_id, to_lanelet_id, ret);
    return ret;
  }
  lanelet::routing::LaneletPath shortest_path = route->shortestPath();
  if (shortest_path.empty()) {
    route_cache_.appendData(from_lanelet_id, to_lanelet_id, ret);
    return ret;
  }
  for (auto lane_itr = shortest_path.begin(); lane_itr != shortest_path.end(); lane_itr++) {
    ret.push_back(lane_itr->id());
  }
  route_cache_.appendData(from_lanelet_id, to_lanelet_id, ret);
  return ret;
}

std::shared_ptr<traffic_simulator::math::CatmullRomSpline> HdMapUtils::getCenterPointsSpline(
  std::int64_t lanelet_id)
{
  getCenterPoints(lanelet_id);
  return center_points_cache_.getCenterPointsSpline(lanelet_id);
}

std::vector<geometry_msgs::msg::Point> HdMapUtils::getCenterPoints(
  std::vector<std::int64_t> lanelet_ids)
{
  std::vector<geometry_msgs::msg::Point> ret;
  if (lanelet_ids.empty()) {
    return ret;
  }
  for (const auto lanelet_id : lanelet_ids) {
    std::vector<geometry_msgs::msg::Point> center_points = getCenterPoints(lanelet_id);
    std::copy(center_points.begin(), center_points.end(), std::back_inserter(ret));
  }
  return ret;
}

std::vector<geometry_msgs::msg::Point> HdMapUtils::getCenterPoints(std::int64_t lanelet_id)
{
  std::vector<geometry_msgs::msg::Point> ret;
  if (!lanelet_map_ptr_) {
    THROW_SIMULATION_ERROR("lanelet map is null pointer");
  }
  if (lanelet_map_ptr_->laneletLayer.empty()) {
    THROW_SIMULATION_ERROR("lanelet layer is empty");
  }
  if (center_points_cache_.exists(lanelet_id)) {
    return center_points_cache_.getCenterPoints(lanelet_id);
  }

  const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
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
  center_points_cache_.appendData(lanelet_id, ret);
  return ret;
}

double HdMapUtils::getLaneletLength(std::int64_t lanelet_id)
{
  if (lanelet_length_cache_.exists(lanelet_id)) {
    return lanelet_length_cache_.getLength(lanelet_id);
  }
  double ret = lanelet::utils::getLaneletLength2d(lanelet_map_ptr_->laneletLayer.get(lanelet_id));
  lanelet_length_cache_.appendData(lanelet_id, ret);
  return ret;
}

std::vector<std::int64_t> HdMapUtils::getPreviousLaneletIds(std::int64_t lanelet_id) const
{
  std::vector<std::int64_t> ret;
  const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  const auto previous_lanelets = vehicle_routing_graph_ptr_->previous(lanelet);
  for (const auto & llt : previous_lanelets) {
    ret.push_back(llt.id());
  }
  return ret;
}

std::vector<std::int64_t> HdMapUtils::getPreviousLaneletIds(
  std::int64_t lanelet_id, std::string turn_direction)
{
  std::vector<std::int64_t> ret;
  const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  const auto previous_lanelets = vehicle_routing_graph_ptr_->previous(lanelet);
  for (const auto & llt : previous_lanelets) {
    const std::string turn_direction_llt = llt.attributeOr("turn_direction", "else");
    if (turn_direction_llt == turn_direction) {
      ret.push_back(llt.id());
    }
  }
  return ret;
}

std::vector<std::int64_t> HdMapUtils::getNextLaneletIds(std::int64_t lanelet_id) const
{
  std::vector<std::int64_t> ret;
  const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  const auto following_lanelets = vehicle_routing_graph_ptr_->following(lanelet);
  for (const auto & llt : following_lanelets) {
    ret.push_back(llt.id());
  }
  return ret;
}

std::vector<std::int64_t> HdMapUtils::getNextLaneletIds(
  std::int64_t lanelet_id, std::string turn_direction)
{
  std::vector<std::int64_t> ret;
  const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  const auto following_lanelets = vehicle_routing_graph_ptr_->following(lanelet);
  for (const auto & llt : following_lanelets) {
    const std::string turn_direction_llt = llt.attributeOr("turn_direction", "else");
    if (turn_direction_llt == turn_direction) {
      ret.push_back(llt.id());
    }
  }
  return ret;
}

const std::vector<std::int64_t> HdMapUtils::getTrafficLightIds() const
{
  std::vector<std::int64_t> ret;
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  auto autoware_traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (const auto light : autoware_traffic_lights) {
    for (auto light_string : light->lightBulbs()) {
      if (light_string.hasAttribute("traffic_light_id")) {
        auto id = light_string.attribute("traffic_light_id").asId();
        if (id) {
          ret.emplace_back(id.get());
        }
      }
    }
  }
  return ret;
}

const boost::optional<geometry_msgs::msg::Point> HdMapUtils::getTrafficLightBulbPosition(
  std::int64_t traffic_light_id, traffic_simulator::TrafficLightColor color) const
{
  if (color == traffic_simulator::TrafficLightColor::NONE) {
    return boost::none;
  }
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  auto autoware_traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (const auto light : autoware_traffic_lights) {
    for (auto light_string : light->lightBulbs()) {
      if (light_string.hasAttribute("traffic_light_id")) {
        auto id = light_string.attribute("traffic_light_id").asId();
        if (id) {
          if (id.get() == traffic_light_id) {
            const auto light_bulbs = light->lightBulbs();
            for (auto ls : light_bulbs) {
              lanelet::ConstLineString3d l = static_cast<lanelet::ConstLineString3d>(ls);
              for (auto pt : l) {
                if (pt.hasAttribute("color")) {
                  std::string color_string;
                  switch (color) {
                    case traffic_simulator::TrafficLightColor::GREEN:
                      color_string = "green";
                      break;
                    case traffic_simulator::TrafficLightColor::YELLOW:
                      color_string = "yellow";
                      break;
                    case traffic_simulator::TrafficLightColor::RED:
                      color_string = "red";
                      break;
                    case traffic_simulator::TrafficLightColor::NONE:
                      return boost::none;
                  }
                  lanelet::Attribute attr = pt.attribute("color");
                  if (attr.value().compare(color_string) == 0) {
                    geometry_msgs::msg::Point point;
                    point.x = pt.x();
                    point.y = pt.y();
                    point.z = pt.z();
                    return point;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  return boost::none;
}

boost::optional<std::pair<traffic_simulator::math::HermiteCurve, double>>
HdMapUtils::getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator::lane_change::Parameter & lane_change_parameter,
  double maximum_curvature_threshold, double target_trajectory_length,
  double forward_distance_threshold)
{
  double to_length = getLaneletLength(lane_change_parameter.target.lanelet_id);
  std::vector<double> evaluation, target_s;
  std::vector<traffic_simulator::math::HermiteCurve> curves;

  for (double to_s = 0; to_s < to_length; to_s = to_s + 1.0) {
    auto goal_pose = toMapPose(lane_change_parameter.target.lanelet_id, to_s, 0);
    if (
      traffic_simulator::math::getRelativePose(from_pose, goal_pose.pose).position.x <=
      forward_distance_threshold) {
      continue;
    }
    double start_to_goal_distance = std::sqrt(
      std::pow(from_pose.position.x - goal_pose.pose.position.x, 2) +
      std::pow(from_pose.position.y - goal_pose.pose.position.y, 2) +
      std::pow(from_pose.position.z - goal_pose.pose.position.z, 2));
    traffic_simulator_msgs::msg::LaneletPose to_pose;
    to_pose.lanelet_id = lane_change_parameter.target.lanelet_id;
    to_pose.s = to_s;
    auto traj = getLaneChangeTrajectory(
      from_pose, to_pose, lane_change_parameter.trajectory_shape, start_to_goal_distance * 0.5);
    if (traj.getMaximum2DCurvature() < maximum_curvature_threshold) {
      double eval = std::fabs(target_trajectory_length - traj.getLength());
      evaluation.push_back(eval);
      curves.push_back(traj);
      target_s.push_back(to_s);
    }
  }
  if (evaluation.empty()) {
    return boost::none;
  }
  std::vector<double>::iterator min_itr = std::min_element(evaluation.begin(), evaluation.end());
  size_t min_index = std::distance(evaluation.begin(), min_itr);
  return std::make_pair(curves[min_index], target_s[min_index]);
}

traffic_simulator::math::HermiteCurve HdMapUtils::getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator_msgs::msg::LaneletPose & to_pose,
  const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
  double tangent_vector_size)
{
  geometry_msgs::msg::Vector3 start_vec;
  geometry_msgs::msg::Vector3 to_vec;
  geometry_msgs::msg::Pose goal_pose =
    toMapPose(to_pose.lanelet_id, to_pose.s, to_pose.offset).pose;
  switch (trajectory_shape) {
    case traffic_simulator::lane_change::TrajectoryShape::CUBIC:
      start_vec = getVectorFromPose(from_pose, tangent_vector_size);
      if (getTangentVector(to_pose.lanelet_id, to_pose.s)) {
        to_vec = getTangentVector(to_pose.lanelet_id, to_pose.s).get();
      } else {
        THROW_SIMULATION_ERROR(
          "Failed to calculate tangent vector at lanelet_id : ", to_pose.lanelet_id,
          " s : ", to_pose.s);
      }
      break;
    case traffic_simulator::lane_change::TrajectoryShape::LINEAR:
      start_vec.x = (goal_pose.position.x - from_pose.position.x);
      start_vec.y = (goal_pose.position.y - from_pose.position.y);
      start_vec.z = (goal_pose.position.z - from_pose.position.z);
      to_vec = start_vec;
      tangent_vector_size = 1;
      break;
  }
  geometry_msgs::msg::Vector3 goal_vec = to_vec;
  goal_vec.x = goal_vec.x * tangent_vector_size;
  goal_vec.y = goal_vec.y * tangent_vector_size;
  goal_vec.z = goal_vec.z * tangent_vector_size;
  traffic_simulator::math::HermiteCurve curve(from_pose, goal_pose, start_vec, goal_vec);
  return curve;
}

geometry_msgs::msg::Vector3 HdMapUtils::getVectorFromPose(
  geometry_msgs::msg::Pose pose, double magnitude)
{
  geometry_msgs::msg::Vector3 dir =
    quaternion_operation::convertQuaternionToEulerAngle(pose.orientation);
  geometry_msgs::msg::Vector3 vector;
  vector.x = magnitude * std::cos(dir.z);
  vector.y = magnitude * std::sin(dir.z);
  vector.z = 0;
  return vector;
}

bool HdMapUtils::isInLanelet(std::int64_t lanelet_id, double s)
{
  const auto spline = getCenterPointsSpline(lanelet_id);
  double l = spline->getLength();
  if (s > l) {
    return false;
  } else if (s < 0) {
    return false;
  }
  return true;
}

std::vector<geometry_msgs::msg::Point> HdMapUtils::toMapPoints(
  std::int64_t lanelet_id, std::vector<double> s)
{
  std::vector<geometry_msgs::msg::Point> ret;
  const auto spline = getCenterPointsSpline(lanelet_id);
  for (const auto & s_value : s) {
    ret.push_back(spline->getPoint(s_value));
  }
  return ret;
}

geometry_msgs::msg::PoseStamped HdMapUtils::toMapPose(
  std::int64_t lanelet_id, double s, double offset, geometry_msgs::msg::Quaternion quat)
{
  geometry_msgs::msg::PoseStamped ret;
  ret.header.frame_id = "map";
  const auto spline = getCenterPointsSpline(lanelet_id);
  ret.pose = spline->getPose(s);
  const auto normal_vec = spline->getNormalVector(s);
  const auto diff = traffic_simulator::math::normalize(normal_vec) * offset;
  ret.pose.position = ret.pose.position + diff;
  const auto tangent_vec = spline->getTangentVector(s);
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = 0.0;
  rpy.y = 0.0;
  rpy.z = std::atan2(tangent_vec.y, tangent_vec.x);
  ret.pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy) * quat;
  return ret;
}

geometry_msgs::msg::PoseStamped HdMapUtils::toMapPose(
  traffic_simulator_msgs::msg::LaneletPose lanelet_pose)
{
  return toMapPose(
    lanelet_pose.lanelet_id, lanelet_pose.s, lanelet_pose.offset,
    quaternion_operation::convertEulerAngleToQuaternion(lanelet_pose.rpy));
}

geometry_msgs::msg::PoseStamped HdMapUtils::toMapPose(
  std::int64_t lanelet_id, double s, double offset)
{
  traffic_simulator_msgs::msg::LaneletPose lanelet_pose;
  lanelet_pose.lanelet_id = lanelet_id;
  lanelet_pose.s = s;
  lanelet_pose.offset = offset;
  return toMapPose(lanelet_pose);
}

boost::optional<geometry_msgs::msg::Vector3> HdMapUtils::getTangentVector(
  std::int64_t lanelet_id, double s)
{
  return getCenterPointsSpline(lanelet_id)->getTangentVector(s);
}

bool HdMapUtils::canChangeLane(std::int64_t from_lanelet_id, std::int64_t to_lanelet_id)
{
  const auto from_lanelet = lanelet_map_ptr_->laneletLayer.get(from_lanelet_id);
  const auto to_lanelet = lanelet_map_ptr_->laneletLayer.get(to_lanelet_id);
  return traffic_rules_vehicle_ptr_->canChangeLane(from_lanelet, to_lanelet);
}

boost::optional<double> HdMapUtils::getLongitudinalDistance(
  traffic_simulator_msgs::msg::LaneletPose from, traffic_simulator_msgs::msg::LaneletPose to)
{
  return getLongitudinalDistance(from.lanelet_id, from.s, to.lanelet_id, to.s);
}

boost::optional<double> HdMapUtils::getLongitudinalDistance(
  std::int64_t from_lanelet_id, double from_s, std::int64_t to_lanelet_id, double to_s)
{
  if (from_lanelet_id == to_lanelet_id) {
    if (from_s > to_s) {
      return boost::none;
    } else {
      return to_s - from_s;
    }
  }
  const auto route = getRoute(from_lanelet_id, to_lanelet_id);
  if (route.empty()) {
    return boost::none;
  }
  double distance = 0;
  for (const auto lanelet_id : route) {
    if (lanelet_id == from_lanelet_id) {
      distance = getLaneletLength(from_lanelet_id) - from_s;
    } else if (lanelet_id == to_lanelet_id) {
      distance = distance + to_s;
    } else {
      distance = distance + getLaneletLength(lanelet_id);
    }
  }
  return distance;
}

const autoware_auto_mapping_msgs::msg::HADMapBin HdMapUtils::toMapBin()
{
  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *lanelet_map_ptr_;
  auto id_counter = lanelet::utils::getId();
  oa << id_counter;
  std::string tmp_str = ss.str();
  autoware_auto_mapping_msgs::msg::HADMapBin msg;
  msg.data.clear();
  msg.data.resize(tmp_str.size());
  msg.data.assign(tmp_str.begin(), tmp_str.end());
  msg.header.frame_id = "map";
  return msg;
}

void HdMapUtils::insertMarkerArray(
  visualization_msgs::msg::MarkerArray & a1, const visualization_msgs::msg::MarkerArray & a2) const
{
  a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}

const visualization_msgs::msg::MarkerArray HdMapUtils::generateMarker() const
{
  visualization_msgs::msg::MarkerArray markers;
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet::ConstLanelets crosswalk_lanelets =
    lanelet::utils::query::crosswalkLanelets(all_lanelets);
  lanelet::ConstLanelets walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);
  std::vector<lanelet::ConstLineString3d> stop_lines =
    lanelet::utils::query::stopLinesLanelets(road_lanelets);
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems =
    lanelet::utils::query::detectionAreas(all_lanelets);
  lanelet::ConstLineStrings3d parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(lanelet_map_ptr_);
  lanelet::ConstPolygons3d parking_lots =
    lanelet::utils::query::getAllParkingLots(lanelet_map_ptr_);

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
    lanelet::visualization::lineStringsAsMarkerArray(stop_lines, "stop_lines", cl_stoplines));
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

void HdMapUtils::overwriteLaneletsCenterline()
{
  for (auto & lanelet_obj : lanelet_map_ptr_->laneletLayer) {
    if (!lanelet_obj.hasCustomCenterline()) {
      const auto fine_center_line = generateFineCenterline(lanelet_obj, 2.0);
      lanelet_obj.setCenterline(fine_center_line);
    }
  }
}

std::pair<size_t, size_t> HdMapUtils::findNearestIndexPair(
  const std::vector<double> & accumulated_lengths, const double target_length)
{
  // List size
  const auto N = accumulated_lengths.size();
  // Front
  if (target_length < accumulated_lengths.at(1)) {
    return std::make_pair(0, 1);
  }
  // Back
  if (target_length > accumulated_lengths.at(N - 2)) {
    return std::make_pair(N - 2, N - 1);
  }

  // Middle
  for (size_t i = 1; i < N; ++i) {
    if (
      accumulated_lengths.at(i - 1) <= target_length &&
      target_length <= accumulated_lengths.at(i)) {
      return std::make_pair(i - 1, i);
    }
  }

  // Throw an exception because this never happens
  THROW_SEMANTIC_ERROR("findNearestIndexPair(): No nearest point found.");
}

const std::unordered_map<std::int64_t, std::vector<std::int64_t>>
HdMapUtils::getRightOfWayLaneletIds(std::vector<std::int64_t> lanelet_ids) const
{
  std::unordered_map<std::int64_t, std::vector<std::int64_t>> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    ret.emplace(lanelet_id, getRightOfWayLaneletIds(lanelet_id));
  }
  return ret;
}

const std::vector<std::int64_t> HdMapUtils::getRightOfWayLaneletIds(std::int64_t lanelet_id) const
{
  std::vector<std::int64_t> ret;
  const auto & assigned_lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  const auto right_of_ways = assigned_lanelet.regulatoryElementsAs<lanelet::RightOfWay>();
  for (const auto & right_of_way : right_of_ways) {
    const auto right_of_Way_lanelets = right_of_way->rightOfWayLanelets();
    for (const auto & ll : right_of_Way_lanelets) {
      ret.emplace_back(ll.id());
    }
  }
  return ret;
}

std::vector<std::shared_ptr<const lanelet::TrafficSign>>
HdMapUtils::getTrafficSignRegElementsOnPath(std::vector<std::int64_t> lanelet_ids) const
{
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const auto traffic_signs = lanelet.regulatoryElementsAs<const lanelet::TrafficSign>();
    for (const auto traffic_sign : traffic_signs) {
      ret.push_back(traffic_sign);
    }
  }
  return ret;
}

std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>>
HdMapUtils::getTrafficLightRegElementsOnPath(const std::vector<std::int64_t> & lanelet_ids) const
{
  std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const auto traffic_lights =
      lanelet.regulatoryElementsAs<const lanelet::autoware::AutowareTrafficLight>();
    for (const auto traffic_light : traffic_lights) {
      ret.push_back(traffic_light);
    }
  }
  return ret;
}

std::vector<lanelet::ConstLineString3d> HdMapUtils::getStopLinesOnPath(
  std::vector<std::int64_t> lanelet_ids)
{
  std::vector<lanelet::ConstLineString3d> ret;
  const auto traffic_signs = getTrafficSignRegElementsOnPath(lanelet_ids);
  for (const auto & traffic_sign : traffic_signs) {
    if (traffic_sign->type() != "stop_sign") {
      continue;
    }
    for (const auto & stop_line : traffic_sign->refLines()) {
      ret.emplace_back(stop_line);
    }
  }
  return ret;
}

std::vector<lanelet::AutowareTrafficLightConstPtr> HdMapUtils::getTrafficLights(
  const std::int64_t traffic_light_id) const
{
  std::vector<lanelet::AutowareTrafficLightConstPtr> ret;
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  auto autoware_traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (const auto light : autoware_traffic_lights) {
    for (auto light_string : light->lightBulbs()) {
      if (light_string.hasAttribute("traffic_light_id")) {
        auto id = light_string.attribute("traffic_light_id").asId();
        if (id == traffic_light_id) {
          ret.emplace_back(light);
        }
      }
    }
  }
  if (ret.empty()) {
    THROW_SEMANTIC_ERROR("traffic_light_id does not match. ID : ", traffic_light_id);
  }
  return ret;
}

std::vector<std::int64_t> HdMapUtils::getTrafficLightStopLineIds(
  const std::int64_t & traffic_light_id) const
{
  std::vector<std::int64_t> ret;
  const auto traffic_lights = getTrafficLights(traffic_light_id);
  for (const auto & traffic_light : traffic_lights) {
    if (traffic_light->stopLine()) {
      ret.emplace_back(traffic_light->stopLine()->id());
    }
  }
  return ret;
}

std::vector<std::vector<geometry_msgs::msg::Point>> HdMapUtils::getTrafficLightStopLinesPoints(
  std::int64_t traffic_light_id) const
{
  std::vector<std::vector<geometry_msgs::msg::Point>> ret;
  const auto traffic_lights = getTrafficLights(traffic_light_id);
  for (const auto & traffic_light : traffic_lights) {
    ret.emplace_back(std::vector<geometry_msgs::msg::Point>{});
    const auto stop_line = traffic_light->stopLine();
    if (stop_line) {
      auto & current_stop_line = ret.back();
      for (const auto point : stop_line.get()) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        current_stop_line.emplace_back(p);
      }
    }
  }
  return ret;
}

const std::vector<geometry_msgs::msg::Point> HdMapUtils::getStopLinePolygon(std::int64_t lanelet_id)
{
  std::vector<geometry_msgs::msg::Point> points;
  const auto stop_line = lanelet_map_ptr_->lineStringLayer.get(lanelet_id);
  for (const auto point : stop_line) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    points.emplace_back(p);
  }
  return points;
}

const std::vector<std::int64_t> HdMapUtils::getTrafficLightIdsOnPath(
  const std::vector<std::int64_t> & route_lanelets) const
{
  std::vector<std::int64_t> ret;
  auto traffic_lights = getTrafficLightRegElementsOnPath(route_lanelets);
  for (const auto traffic_light : traffic_lights) {
    for (auto light_string : traffic_light->lightBulbs()) {
      if (light_string.hasAttribute("traffic_light_id")) {
        auto id = light_string.attribute("traffic_light_id").asId();
        if (id) {
          ret.emplace_back(id.get());
        }
      }
    }
  }
  return ret;
}

const boost::optional<double> HdMapUtils::getDistanceToTrafficLightStopLine(
  const std::vector<std::int64_t> & route_lanelets,
  const std::vector<geometry_msgs::msg::Point> & waypoints) const
{
  auto traffic_light_ids = getTrafficLightIdsOnPath(route_lanelets);
  if (traffic_light_ids.size() == 0) {
    return boost::none;
  }
  std::set<double> collision_points;
  for (const auto id : traffic_light_ids) {
    const auto collision_point = getDistanceToTrafficLightStopLine(waypoints, id);
    if (collision_point) {
      collision_points.insert(collision_point.get());
    }
  }
  if (collision_points.empty()) {
    return boost::none;
  }
  return *collision_points.begin();
}

const boost::optional<double> HdMapUtils::getDistanceToTrafficLightStopLine(
  const std::vector<geometry_msgs::msg::Point> & waypoints,
  const std::int64_t & traffic_light_id) const
{
  if (waypoints.empty()) {
    return boost::none;
  }
  traffic_simulator::math::CatmullRomSpline spline(waypoints);
  const auto stop_lines = getTrafficLightStopLinesPoints(traffic_light_id);
  for (const auto & stop_line : stop_lines) {
    const auto collision_point = spline.getCollisionPointIn2D(stop_line);
    if (collision_point) {
      return collision_point;
    }
  }
  return boost::none;
}

boost::optional<double> HdMapUtils::getDistanceToStopLine(
  const std::vector<std::int64_t> & route_lanelets,
  const std::vector<geometry_msgs::msg::Point> & waypoints)
{
  if (waypoints.empty()) {
    return boost::none;
  }
  std::set<double> collision_points;
  if (waypoints.empty()) {
    return boost::none;
  }
  traffic_simulator::math::CatmullRomSpline spline(waypoints);
  const auto stop_lines = getStopLinesOnPath({route_lanelets});
  for (const auto & stop_line : stop_lines) {
    std::vector<geometry_msgs::msg::Point> stop_line_points;
    for (const auto & point : stop_line) {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      stop_line_points.emplace_back(p);
    }
    const auto collision_point = spline.getCollisionPointIn2D(stop_line_points);
    if (collision_point) {
      collision_points.insert(collision_point.get());
    }
  }
  if (collision_points.empty()) {
    return boost::none;
  }
  return *collision_points.begin();
}

std::vector<double> HdMapUtils::calculateSegmentDistances(
  const lanelet::ConstLineString3d & line_string)
{
  std::vector<double> segment_distances;
  segment_distances.reserve(line_string.size() - 1);
  for (size_t i = 1; i < line_string.size(); ++i) {
    const auto distance = lanelet::geometry::distance(line_string[i], line_string[i - 1]);
    segment_distances.push_back(distance);
  }
  return segment_distances;
}

std::vector<double> HdMapUtils::calculateAccumulatedLengths(
  const lanelet::ConstLineString3d & line_string)
{
  const auto segment_distances = calculateSegmentDistances(line_string);

  std::vector<double> accumulated_lengths{0};
  accumulated_lengths.reserve(segment_distances.size() + 1);
  std::partial_sum(
    std::begin(segment_distances), std::end(segment_distances),
    std::back_inserter(accumulated_lengths));
  return accumulated_lengths;
}

std::vector<lanelet::BasicPoint3d> HdMapUtils::resamplePoints(
  const lanelet::ConstLineString3d & line_string, const int32_t num_segments)
{
  // Calculate length
  const auto line_length = lanelet::geometry::length(line_string);

  // Calculate accumulated lengths
  const auto accumulated_lengths = calculateAccumulatedLengths(line_string);

  // Create each segment
  std::vector<lanelet::BasicPoint3d> resampled_points;
  for (auto i = 0; i <= num_segments; ++i) {
    // Find two nearest points
    const double target_length =
      (static_cast<double>(i) / num_segments) * static_cast<double>(line_length);
    const auto index_pair = findNearestIndexPair(accumulated_lengths, target_length);

    // Apply linear interpolation
    const lanelet::BasicPoint3d back_point = line_string[index_pair.first];
    const lanelet::BasicPoint3d front_point = line_string[index_pair.second];
    const auto direction_vector = (front_point - back_point);

    const auto back_length = accumulated_lengths.at(index_pair.first);
    const auto front_length = accumulated_lengths.at(index_pair.second);
    const auto segment_length = front_length - back_length;
    const auto target_point =
      back_point + (direction_vector * (target_length - back_length) / segment_length);

    // Add to list
    resampled_points.push_back(target_point);
  }
  return resampled_points;
}

lanelet::LineString3d HdMapUtils::generateFineCenterline(
  const lanelet::ConstLanelet & lanelet_obj, const double resolution)
{
  // Get length of longer border
  const double left_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.leftBound()));
  const double right_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.rightBound()));
  const double longer_distance = (left_length > right_length) ? left_length : right_length;
  const int32_t num_segments =
    std::max(static_cast<int32_t>(ceil(longer_distance / resolution)), 1);

  // Resample points
  const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create centerline
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (size_t i = 0; i < static_cast<size_t>(num_segments + 1); i++) {
    // Add ID for the average point of left and right
    const auto center_basic_point = (right_points.at(i) + left_points.at(i)) / 2.0;
    const lanelet::Point3d center_point(
      lanelet::utils::getId(), center_basic_point.x(), center_basic_point.y(),
      center_basic_point.z());
    centerline.push_back(center_point);
  }
  return centerline;
}

std::vector<double> HdMapUtils::calcEuclidDist(
  const std::vector<double> & x, const std::vector<double> & y, const std::vector<double> & z)
{
  std::vector<double> dist_v;
  dist_v.push_back(0.0);
  for (size_t i = 0; i < x.size() - 1; ++i) {
    const double dx = x.at(i + 1) - x.at(i);
    const double dy = y.at(i + 1) - y.at(i);
    const double dz = z.at(i + 1) - z.at(i);
    const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    dist_v.push_back(dist_v.at(i) + d);
  }
  return dist_v;
}

std::vector<lanelet::Lanelet> HdMapUtils::getLanelets(
  const std::vector<std::int64_t> & lanelet_ids) const
{
  std::vector<lanelet::Lanelet> lanelets;
  for (const auto & id : lanelet_ids) {
    lanelets.emplace_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  return lanelets;
}

std::vector<std::int64_t> HdMapUtils::getLaneletIds(
  const std::vector<lanelet::Lanelet> & lanelets) const
{
  std::vector<std::int64_t> ids;
  for (const auto & lanelet : lanelets) {
    ids.emplace_back(lanelet.id());
  }
  return ids;
}
}  // namespace hdmap_utils
