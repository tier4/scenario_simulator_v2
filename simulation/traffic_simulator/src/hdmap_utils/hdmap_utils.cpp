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

#include <lanelet2_core/primitives/Primitive.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_projection/UTM.h>

#include <algorithm>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <deque>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/operator.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/spline/hermite_curve.hpp>
#include <geometry/transform.hpp>
#include <geometry/vector3/inner_product.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <memory>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/lanelet_wrapper/traffic_rules.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hdmap_utils
{
using namespace traffic_simulator::lanelet_wrapper;

HdMapUtils::HdMapUtils(
  const boost::filesystem::path & lanelet2_map_path, const geographic_msgs::msg::GeoPoint &)
{
  lanelet::projection::MGRSProjector projector;

  lanelet::ErrorMessages errors;

  lanelet_map_ptr_ = lanelet::load(lanelet2_map_path.string(), projector, &errors);

  routing_graphs_ = std::make_unique<RoutingGraphs>(lanelet_map_ptr_);

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
}

auto HdMapUtils::countLaneChanges(
  const traffic_simulator_msgs::msg::LaneletPose & from,
  const traffic_simulator_msgs::msg::LaneletPose & to,
  const traffic_simulator::RoutingConfiguration & routing_configuration) const
  -> std::optional<std::pair<int, int>>
{
  /// @note a lane change considers the lanes in the same direction as the original, so ignore the lanes in the opposite direction
  constexpr bool include_opposite_direction{false};
  const auto route = getRoute(from.lanelet_id, to.lanelet_id, routing_configuration);
  if (route.empty()) {
    return std::nullopt;
  } else {
    std::pair<int, int> lane_changes{0, 0};
    for (std::size_t i = 1; i < route.size(); ++i) {
      const auto & previous = route[i - 1];
      const auto & current = route[i];
      if (const auto followings =
            lanelet_map::nextLaneletIds(previous, routing_configuration.routing_graph_type);
          std::find(followings.begin(), followings.end(), current) != followings.end()) {
        continue;
      } else if (const auto lefts = pose::leftLaneletIds(
                   previous, routing_configuration.routing_graph_type, include_opposite_direction);
                 std::find(lefts.begin(), lefts.end(), current) != lefts.end()) {
        lane_changes.first++;
      } else if (const auto rights = pose::rightLaneletIds(
                   previous, routing_configuration.routing_graph_type, include_opposite_direction);
                 std::find(rights.begin(), rights.end(), current) != rights.end()) {
        lane_changes.second++;
      }
    }
    return lane_changes;
  }
}

auto HdMapUtils::getLaneletIds() const -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
    ids.push_back(lanelet.id());
  }
  return ids;
}

auto HdMapUtils::getLaneletPolygon(const lanelet::Id lanelet_id) const
  -> std::vector<geometry_msgs::msg::Point>
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

auto HdMapUtils::filterLaneletIds(const lanelet::Ids & lanelet_ids, const char subtype[]) const
  -> lanelet::Ids
{
  const auto lanelets = getLanelets(lanelet_ids);
  lanelet::Lanelets filtered_lanelets;
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

auto HdMapUtils::getNearbyLaneletIds(
  const geometry_msgs::msg::Point & position, const double distance_threshold,
  const std::size_t search_count) const -> lanelet::Ids
{
  lanelet::Ids lanelet_ids;
  lanelet::BasicPoint2d search_point(position.x, position.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, search_count);
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

auto HdMapUtils::getNearbyLaneletIds(
  const geometry_msgs::msg::Point & point, const double distance_thresh,
  const bool include_crosswalk, const std::size_t search_count) const -> lanelet::Ids
{
  lanelet::Ids lanelet_ids;
  lanelet::BasicPoint2d search_point(point.x, point.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, search_count);
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

auto HdMapUtils::getAltitude(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose) const
  -> double
{
  return pose::toMapPose(lanelet_pose).pose.position.z;
}

auto HdMapUtils::getCollisionPointInLaneCoordinate(
  const lanelet::Id lanelet_id, const lanelet::Id crossing_lanelet_id) const
  -> std::optional<double>
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
  return std::nullopt;
}

auto HdMapUtils::getConflictingLaneIds(
  const lanelet::Ids & lanelet_ids, const traffic_simulator::RoutingGraphType type) const
  -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const auto conflicting_lanelets =
      lanelet::utils::getConflictingLanelets(routing_graphs_->routing_graph(type), lanelet);
    for (const auto & conflicting_lanelet : conflicting_lanelets) {
      ids.emplace_back(conflicting_lanelet.id());
    }
  }
  return ids;
}

auto HdMapUtils::getConflictingCrosswalkIds(const lanelet::Ids & lanelet_ids) const -> lanelet::Ids
{
  lanelet::Ids ids;
  std::vector<lanelet::routing::RoutingGraphConstPtr> graphs;
  graphs.emplace_back(routing_graphs_->routing_graph(
    traffic_simulator::RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER));
  graphs.emplace_back(
    routing_graphs_->routing_graph(traffic_simulator::RoutingGraphType::PEDESTRIAN));
  lanelet::routing::RoutingGraphContainer container(graphs);
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
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

auto HdMapUtils::clipTrajectoryFromLaneletIds(
  const lanelet::Id lanelet_id, const double s, const lanelet::Ids & lanelet_ids,
  const double forward_distance) const -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> ret;
  bool on_traj = false;
  double rest_distance = forward_distance;
  for (auto id_itr = lanelet_ids.begin(); id_itr != lanelet_ids.end(); id_itr++) {
    double l = lanelet_map::laneletLength(*id_itr);
    if (on_traj) {
      if (rest_distance < l) {
        for (double s_val = 0; s_val < rest_distance; s_val = s_val + 1.0) {
          ret.emplace_back(
            pose::toMapPose(traffic_simulator::helper::constructLaneletPose(*id_itr, s_val, 0))
              .pose.position);
        }
        break;
      } else {
        rest_distance = rest_distance - l;
        for (double s_val = 0; s_val < l; s_val = s_val + 1.0) {
          ret.emplace_back(
            pose::toMapPose(traffic_simulator::helper::constructLaneletPose(*id_itr, s_val, 0.0))
              .pose.position);
        }
        continue;
      }
    }
    if (lanelet_id == *id_itr) {
      on_traj = true;
      if ((s + forward_distance) < l) {
        for (double s_val = s; s_val < s + forward_distance; s_val = s_val + 1.0) {
          ret.emplace_back(
            pose::toMapPose(traffic_simulator::helper::constructLaneletPose(lanelet_id, s_val, 0.0))
              .pose.position);
        }
        break;
      } else {
        rest_distance = rest_distance - (l - s);
        for (double s_val = s; s_val < l; s_val = s_val + 1.0) {
          ret.emplace_back(
            pose::toMapPose(traffic_simulator::helper::constructLaneletPose(lanelet_id, s_val, 0.0))
              .pose.position);
        }
        continue;
      }
    }
  }
  return ret;
}

auto HdMapUtils::filterLanelets(const lanelet::Lanelets & lanelets, const char subtype[]) const
  -> lanelet::Lanelets
{
  lanelet::Lanelets filtered_lanelets;
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

auto HdMapUtils::excludeSubtypeLanelets(
  const std::vector<std::pair<double, lanelet::Lanelet>> & lls, const char subtype[]) const
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

auto HdMapUtils::absoluteHull(
  const lanelet::BasicPolygon2d & relativeHull, const lanelet::matching::Pose2d & pose) const
  -> lanelet::BasicPolygon2d
{
  lanelet::BasicPolygon2d hullPoints;
  hullPoints.reserve(relativeHull.size());
  for (const auto & hullPt : relativeHull) {
    hullPoints.push_back(pose * hullPt);
  }
  return hullPoints;
}

auto HdMapUtils::toPoint2d(const geometry_msgs::msg::Point & point) const -> lanelet::BasicPoint2d
{
  return lanelet::BasicPoint2d{point.x, point.y};
}

auto HdMapUtils::getClosestLaneletId(
  const geometry_msgs::msg::Pose & pose, const double distance_thresh,
  const bool include_crosswalk) const -> std::optional<lanelet::Id>
{
  lanelet::BasicPoint2d search_point(pose.position.x, pose.position.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 3);
  if (include_crosswalk) {
    if (nearest_lanelet.empty()) {
      return std::nullopt;
    }
    if (nearest_lanelet.front().first > distance_thresh) {
      return std::nullopt;
    }
    lanelet::Lanelet closest_lanelet;
    closest_lanelet = nearest_lanelet.front().second;
    return closest_lanelet.id();
  } else {
    const auto nearest_road_lanelet =
      excludeSubtypeLanelets(nearest_lanelet, lanelet::AttributeValueString::Crosswalk);
    if (nearest_road_lanelet.empty()) {
      return std::nullopt;
    }
    if (nearest_road_lanelet.front().first > distance_thresh) {
      return std::nullopt;
    }
    lanelet::Lanelet closest_lanelet;
    closest_lanelet = nearest_road_lanelet.front().second;
    return closest_lanelet.id();
  }
}

auto HdMapUtils::getSpeedLimit(
  const lanelet::Ids & lanelet_ids, const traffic_simulator::RoutingGraphType type) const -> double
{
  std::vector<double> limits;
  if (lanelet_ids.empty()) {
    THROW_SEMANTIC_ERROR("size of the vector lanelet ids should be more than 1");
  }
  for (auto itr = lanelet_ids.begin(); itr != lanelet_ids.end(); itr++) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(*itr);
    const auto limit = routing_graphs_->traffic_rule(type)->speedLimit(lanelet);
    limits.push_back(lanelet::units::KmHQuantity(limit.speedLimit).value() / 3.6);
  }
  return *std::min_element(limits.begin(), limits.end());
}

auto HdMapUtils::getLaneChangeableLaneletId(
  const lanelet::Id lanelet_id, const traffic_simulator::lane_change::Direction direction,
  const std::uint8_t shift, const traffic_simulator::RoutingGraphType type) const
  -> std::optional<lanelet::Id>
{
  if (shift == 0) {
    return getLaneChangeableLaneletId(
      lanelet_id, traffic_simulator::lane_change::Direction::STRAIGHT, type);
  } else {
    auto reference_id = lanelet_id;
    for (uint8_t i = 0; i < shift; i++) {
      auto id = getLaneChangeableLaneletId(reference_id, direction, type);
      if (!id) {
        return std::nullopt;
      } else {
        reference_id = id.value();
      }
      if (i == (shift - 1)) {
        return reference_id;
      }
    }
  }
  return std::nullopt;
}

auto HdMapUtils::getLaneChangeableLaneletId(
  const lanelet::Id lanelet_id, const traffic_simulator::lane_change::Direction direction,
  const traffic_simulator::RoutingGraphType type) const -> std::optional<lanelet::Id>
{
  const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
  std::optional<lanelet::Id> target = std::nullopt;
  switch (direction) {
    case traffic_simulator::lane_change::Direction::STRAIGHT:
      target = lanelet.id();
      break;
    case traffic_simulator::lane_change::Direction::LEFT:
      if (routing_graphs_->routing_graph(type)->left(lanelet)) {
        target = routing_graphs_->routing_graph(type)->left(lanelet)->id();
      }
      break;
    case traffic_simulator::lane_change::Direction::RIGHT:
      if (routing_graphs_->routing_graph(type)->right(lanelet)) {
        target = routing_graphs_->routing_graph(type)->right(lanelet)->id();
      }
      break;
  }
  return target;
}

auto HdMapUtils::getPreviousLanelets(
  const lanelet::Id current_lanelet_id, const double backward_horizon,
  const traffic_simulator::RoutingGraphType type) const -> lanelet::Ids
{
  lanelet::Ids previous_lanelets_ids;
  double total_distance = 0.0;
  previous_lanelets_ids.push_back(current_lanelet_id);
  while (total_distance < backward_horizon) {
    const auto & reference_lanelet_id = previous_lanelets_ids.back();
    if (const auto straight_lanelet_ids =
          lanelet_map::previousLaneletIds(reference_lanelet_id, "straight", type);
        not straight_lanelet_ids.empty()) {
      total_distance = total_distance + lanelet_map::laneletLength(straight_lanelet_ids[0]);
      previous_lanelets_ids.push_back(straight_lanelet_ids[0]);
    } else if (auto non_straight_lanelet_ids =
                 lanelet_map::previousLaneletIds(reference_lanelet_id, type);
               not non_straight_lanelet_ids.empty()) {
      total_distance = total_distance + lanelet_map::laneletLength(non_straight_lanelet_ids[0]);
      previous_lanelets_ids.push_back(non_straight_lanelet_ids[0]);
    } else {
      break;
    }
  }
  return previous_lanelets_ids;
}

auto HdMapUtils::isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route) const -> bool
{
  return std::find(route.cbegin(), route.cend(), lanelet_id) != route.cend();
}

auto HdMapUtils::getFollowingLanelets(
  const lanelet::Id current_lanelet_id, const lanelet::Ids & route, const double horizon,
  const bool include_current_lanelet_id, const traffic_simulator::RoutingGraphType type) const
  -> lanelet::Ids
{
  const auto is_following_lanelet =
    [this, type](const auto & current_lanelet, const auto & candidate_lanelet) {
      const auto next_ids = lanelet_map::nextLaneletIds(current_lanelet, type);
      return std::find(next_ids.cbegin(), next_ids.cend(), candidate_lanelet) != next_ids.cend();
    };

  lanelet::Ids following_lanelets_ids;

  if (route.empty()) {
    return following_lanelets_ids;
  }

  double total_distance = 0.0;
  bool found_starting_lanelet = false;

  for (const auto & candidate_lanelet_id : route) {
    if (found_starting_lanelet) {
      if (const auto previous_lanelet =
            following_lanelets_ids.empty() ? current_lanelet_id : following_lanelets_ids.back();
          not is_following_lanelet(previous_lanelet, candidate_lanelet_id)) {
        THROW_SEMANTIC_ERROR(
          candidate_lanelet_id + " is not the follower of lanelet " + previous_lanelet);
      }
      following_lanelets_ids.push_back(candidate_lanelet_id);
      total_distance += lanelet_map::laneletLength(candidate_lanelet_id);
      if (total_distance > horizon) {
        break;
      }
    } else if (candidate_lanelet_id == current_lanelet_id) {
      found_starting_lanelet = true;
      if (include_current_lanelet_id) {
        following_lanelets_ids.push_back(candidate_lanelet_id);
      }
    }
  }
  if (following_lanelets_ids.empty()) {
    THROW_SEMANTIC_ERROR("lanelet id does not match");
  } else if (total_distance <= horizon) {
    const auto remaining_lanelets =
      getFollowingLanelets(route.back(), horizon - total_distance, false, type);
    following_lanelets_ids.insert(
      following_lanelets_ids.end(), remaining_lanelets.begin(), remaining_lanelets.end());
  }

  return following_lanelets_ids;
}

auto HdMapUtils::getFollowingLanelets(
  const lanelet::Id lanelet_id, const double distance, const bool include_self,
  const traffic_simulator::RoutingGraphType type) const -> lanelet::Ids
{
  lanelet::Ids ret;
  double total_distance = 0.0;
  if (include_self) {
    ret.push_back(lanelet_id);
  }
  lanelet::Id end_lanelet_id = lanelet_id;
  while (total_distance < distance) {
    if (const auto straight_ids = lanelet_map::nextLaneletIds(end_lanelet_id, "straight", type);
        !straight_ids.empty()) {
      total_distance = total_distance + lanelet_map::laneletLength(straight_ids[0]);
      ret.push_back(straight_ids[0]);
      end_lanelet_id = straight_ids[0];
      continue;
    } else if (const auto ids = lanelet_map::nextLaneletIds(end_lanelet_id, type);
               ids.size() != 0) {
      total_distance = total_distance + lanelet_map::laneletLength(ids[0]);
      ret.push_back(ids[0]);
      end_lanelet_id = ids[0];
      continue;
    } else {
      break;
    }
  }
  return ret;
}

auto HdMapUtils::getRoute(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id,
  const traffic_simulator::RoutingConfiguration & routing_configuration) const -> lanelet::Ids
{
  return routing_graphs_->getRoute(
    from_lanelet_id, to_lanelet_id, lanelet_map_ptr_, routing_configuration);
}

auto HdMapUtils::getCenterPointsSpline(const lanelet::Id lanelet_id) const
  -> std::shared_ptr<math::geometry::CatmullRomSpline>
{
  getCenterPoints(lanelet_id);
  return center_points_cache_.getCenterPointsSpline(lanelet_id);
}

auto HdMapUtils::getCenterPoints(const lanelet::Ids & lanelet_ids) const
  -> std::vector<geometry_msgs::msg::Point>
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

auto HdMapUtils::getCenterPoints(const lanelet::Id lanelet_id) const
  -> std::vector<geometry_msgs::msg::Point>
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

auto HdMapUtils::getTrafficLightIds() const -> lanelet::Ids
{
  using namespace lanelet::utils::query;

  lanelet::Ids ids;

  for (auto && traffic_light : autowareTrafficLights(laneletLayer(lanelet_map_ptr_))) {
    for (auto && light_bulb : traffic_light->lightBulbs()) {
      if (light_bulb.hasAttribute("traffic_light_id")) {
        if (auto id = light_bulb.attribute("traffic_light_id").asId()) {
          ids.emplace_back(id.value());
        }
      }
    }
  }

  return ids;
}

auto HdMapUtils::getTrafficLightBulbPosition(
  const lanelet::Id traffic_light_id, const std::string & color_name) const
  -> std::optional<geometry_msgs::msg::Point>
{
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  auto autoware_traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);

  auto areBulbsAssignedToTrafficLight = [traffic_light_id](auto red_yellow_green_bulbs) -> bool {
    return red_yellow_green_bulbs.hasAttribute("traffic_light_id") and
           red_yellow_green_bulbs.attribute("traffic_light_id").asId() and
           red_yellow_green_bulbs.attribute("traffic_light_id").asId().value() == traffic_light_id;
  };

  auto isBulbOfExpectedColor = [color_name](auto bulb) -> bool {
    return bulb.hasAttribute("color") and !bulb.hasAttribute("arrow") and
           bulb.attribute("color").value().compare(color_name) == 0;
  };

  for (const auto & light : autoware_traffic_lights) {
    for (auto three_light_bulbs : light->lightBulbs()) {
      if (areBulbsAssignedToTrafficLight(three_light_bulbs)) {
        for (auto bulb : static_cast<lanelet::ConstLineString3d>(three_light_bulbs)) {
          if (isBulbOfExpectedColor(bulb)) {
            geometry_msgs::msg::Point point;
            point.x = bulb.x();
            point.y = bulb.y();
            point.z = bulb.z();
            return point;
          }
        }
      }
    }
  }
  return std::nullopt;
}

auto HdMapUtils::getLaneChangeTrajectory(
  const traffic_simulator_msgs::msg::LaneletPose & from_pose,
  const traffic_simulator::lane_change::Parameter & lane_change_parameter) const
  -> std::optional<std::pair<math::geometry::HermiteCurve, double>>
{
  double longitudinal_distance =
    traffic_simulator::lane_change::Parameter::default_lanechange_distance;
  switch (lane_change_parameter.constraint.type) {
    case traffic_simulator::lane_change::Constraint::Type::NONE:
      longitudinal_distance =
        traffic_simulator::lane_change::Parameter::default_lanechange_distance;
      break;
    case traffic_simulator::lane_change::Constraint::Type::LATERAL_VELOCITY:
      longitudinal_distance =
        traffic_simulator::lane_change::Parameter::default_lanechange_distance;
      break;
    case traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE:
      longitudinal_distance = lane_change_parameter.constraint.value;
      break;
    case traffic_simulator::lane_change::Constraint::Type::TIME:
      longitudinal_distance =
        traffic_simulator::lane_change::Parameter::default_lanechange_distance;
      break;
  }
  const auto along_pose = pose::alongLaneletPose(from_pose, longitudinal_distance);
  // clang-format off
  const auto left_point =
    pose::toMapPose(traffic_simulator::helper::constructLaneletPose(
      along_pose.lanelet_id, along_pose.s, along_pose.offset + 5.0)).pose.position;
  const auto right_point =
    pose::toMapPose(traffic_simulator::helper::constructLaneletPose(
      along_pose.lanelet_id, along_pose.s, along_pose.offset - 5.0)).pose.position;
  // clang-format on
  const auto collision_point = getCenterPointsSpline(lane_change_parameter.target.lanelet_id)
                                 ->getCollisionPointIn2D(left_point, right_point);
  if (!collision_point) {
    return std::nullopt;
  }
  const auto to_pose = traffic_simulator::helper::constructLaneletPose(
    lane_change_parameter.target.lanelet_id, collision_point.value(),
    lane_change_parameter.target.offset);
  const auto goal_pose_in_map = pose::toMapPose(to_pose).pose;
  const auto from_pose_in_map = pose::toMapPose(from_pose).pose;
  double start_to_goal_distance = std::sqrt(
    std::pow(from_pose_in_map.position.x - goal_pose_in_map.position.x, 2) +
    std::pow(from_pose_in_map.position.y - goal_pose_in_map.position.y, 2) +
    std::pow(from_pose_in_map.position.z - goal_pose_in_map.position.z, 2));

  auto traj = getLaneChangeTrajectory(
    pose::toMapPose(from_pose).pose, to_pose, lane_change_parameter.trajectory_shape,
    start_to_goal_distance * 0.5);
  return std::make_pair(traj, collision_point.value());
}

auto HdMapUtils::getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator::lane_change::Parameter & lane_change_parameter,
  const double maximum_curvature_threshold, const double target_trajectory_length,
  const double forward_distance_threshold) const
  -> std::optional<std::pair<math::geometry::HermiteCurve, double>>
{
  double to_length = lanelet_map::laneletLength(lane_change_parameter.target.lanelet_id);
  std::vector<double> evaluation, target_s;
  std::vector<math::geometry::HermiteCurve> curves;

  for (double to_s = 0; to_s < to_length; to_s = to_s + 1.0) {
    auto goal_pose = pose::toMapPose(traffic_simulator::helper::constructLaneletPose(
      lane_change_parameter.target.lanelet_id, to_s));
    if (
      math::geometry::getRelativePose(from_pose, goal_pose.pose).position.x <=
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
    return std::nullopt;
  }
  std::vector<double>::iterator min_itr = std::min_element(evaluation.begin(), evaluation.end());
  size_t min_index = std::distance(evaluation.begin(), min_itr);
  return std::make_pair(curves[min_index], target_s[min_index]);
}

auto HdMapUtils::getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator_msgs::msg::LaneletPose & to_pose,
  const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
  const double tangent_vector_size) const -> math::geometry::HermiteCurve
{
  geometry_msgs::msg::Vector3 start_vec;
  geometry_msgs::msg::Vector3 to_vec;
  geometry_msgs::msg::Pose goal_pose = pose::toMapPose(to_pose).pose;
  double tangent_vector_size_in_curve = 0.0;
  switch (trajectory_shape) {
    case traffic_simulator::lane_change::TrajectoryShape::CUBIC:
      start_vec = getVectorFromPose(from_pose, tangent_vector_size);
      if (getTangentVector(to_pose.lanelet_id, to_pose.s)) {
        to_vec = getTangentVector(to_pose.lanelet_id, to_pose.s).value();
      } else {
        THROW_SIMULATION_ERROR(
          "Failed to calculate tangent vector at lanelet_id : ", to_pose.lanelet_id,
          " s : ", to_pose.s);
      }
      tangent_vector_size_in_curve = tangent_vector_size;
      break;
    case traffic_simulator::lane_change::TrajectoryShape::LINEAR:
      start_vec.x = (goal_pose.position.x - from_pose.position.x);
      start_vec.y = (goal_pose.position.y - from_pose.position.y);
      start_vec.z = (goal_pose.position.z - from_pose.position.z);
      to_vec = start_vec;
      tangent_vector_size_in_curve = 1;
      break;
  }
  return math::geometry::HermiteCurve(
    from_pose, goal_pose, start_vec,
    geometry_msgs::build<geometry_msgs::msg::Vector3>()
      .x(to_vec.x * tangent_vector_size_in_curve)
      .y(to_vec.y * tangent_vector_size_in_curve)
      .z(to_vec.z * tangent_vector_size_in_curve));
}

auto HdMapUtils::getVectorFromPose(const geometry_msgs::msg::Pose & pose, const double magnitude)
  const -> geometry_msgs::msg::Vector3
{
  geometry_msgs::msg::Vector3 dir = math::geometry::convertQuaternionToEulerAngle(pose.orientation);
  geometry_msgs::msg::Vector3 vector;
  vector.x = magnitude * std::cos(dir.z);
  vector.y = magnitude * std::sin(dir.z);
  vector.z = 0;
  return vector;
}

auto HdMapUtils::isInIntersection(const lanelet::Id lanelet_id) const -> bool
{
  return lanelet_map_ptr_->laneletLayer.get(lanelet_id).hasAttribute("turn_direction");
}

auto HdMapUtils::isInLanelet(const lanelet::Id lanelet_id, const double s) const -> bool
{
  return 0 <= s and s <= getCenterPointsSpline(lanelet_id)->getLength();
}

auto HdMapUtils::toMapPoints(const lanelet::Id lanelet_id, const std::vector<double> & s) const
  -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> ret;
  const auto spline = getCenterPointsSpline(lanelet_id);
  for (const auto & s_value : s) {
    ret.push_back(spline->getPoint(s_value));
  }
  return ret;
}

auto HdMapUtils::getTangentVector(const lanelet::Id lanelet_id, const double s) const
  -> std::optional<geometry_msgs::msg::Vector3>
{
  return getCenterPointsSpline(lanelet_id)->getTangentVector(s);
}

auto HdMapUtils::canChangeLane(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id,
  const traffic_simulator::RoutingGraphType type) const -> bool
{
  const auto from_lanelet = lanelet_map_ptr_->laneletLayer.get(from_lanelet_id);
  const auto to_lanelet = lanelet_map_ptr_->laneletLayer.get(to_lanelet_id);
  return routing_graphs_->traffic_rule(type)->canChangeLane(from_lanelet, to_lanelet);
}

auto HdMapUtils::getLateralDistance(
  const traffic_simulator_msgs::msg::LaneletPose & from,
  const traffic_simulator_msgs::msg::LaneletPose & to,
  const traffic_simulator::RoutingConfiguration & routing_configuration) const
  -> std::optional<double>
{
  const auto route = getRoute(from.lanelet_id, to.lanelet_id, routing_configuration);
  if (route.empty()) {
    return std::nullopt;
  }
  if (routing_configuration.allow_lane_change) {
    double lateral_distance_by_lane_change = 0.0;
    for (unsigned int i = 0; i < route.size() - 1; i++) {
      auto next_lanelet_ids =
        lanelet_map::nextLaneletIds(route[i], routing_configuration.routing_graph_type);
      if (auto next_lanelet = std::find_if(
            next_lanelet_ids.begin(), next_lanelet_ids.end(),
            [&route, i](const lanelet::Id & id) { return id == route[i + 1]; });
          next_lanelet == next_lanelet_ids.end()) {
        traffic_simulator_msgs::msg::LaneletPose next_lanelet_pose;
        next_lanelet_pose.lanelet_id = route[i + 1];
        next_lanelet_pose.s = 0.0;
        next_lanelet_pose.offset = 0.0;

        if (
          auto next_lanelet_origin_from_current_lanelet =
            pose::toLaneletPose(pose::toMapPose(next_lanelet_pose).pose, route[i], 10.0)) {
          lateral_distance_by_lane_change += next_lanelet_origin_from_current_lanelet->offset;
        } else {
          traffic_simulator_msgs::msg::LaneletPose current_lanelet_pose = next_lanelet_pose;
          current_lanelet_pose.lanelet_id = route[i];
          if (
            auto current_lanelet_origin_from_next_lanelet =
              pose::toLaneletPose(pose::toMapPose(current_lanelet_pose).pose, route[i + 1], 10.0)) {
            lateral_distance_by_lane_change -= current_lanelet_origin_from_next_lanelet->offset;
          } else {
            return std::nullopt;
          }
        }
      }
    }
    return to.offset - from.offset + lateral_distance_by_lane_change;
  } else {
    return to.offset - from.offset;
  }
}

auto HdMapUtils::getLongitudinalDistance(
  const traffic_simulator_msgs::msg::LaneletPose & from_pose,
  const traffic_simulator_msgs::msg::LaneletPose & to_pose,
  const traffic_simulator::RoutingConfiguration & routing_configuration) const
  -> std::optional<double>
{
  const auto is_lane_change_required = [routing_configuration](
                                         const lanelet::Id current_lanelet,
                                         const lanelet::Id next_lanelet) -> bool {
    const auto next_lanelet_ids =
      lanelet_map::nextLaneletIds(current_lanelet, routing_configuration.routing_graph_type);
    return std::none_of(
      next_lanelet_ids.cbegin(), next_lanelet_ids.cend(),
      [next_lanelet](const lanelet::Id id) { return id == next_lanelet; });
  };

  /// @sa https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/DistanceCalculation/
  if (const auto route = getRoute(from_pose.lanelet_id, to_pose.lanelet_id, routing_configuration);
      not route.empty() || from_pose.lanelet_id == to_pose.lanelet_id) {
    /// @note horizontal bar must intersect with the lanelet spline, so the distance was set arbitrarily to 10 meters.
    static constexpr double matching_distance = 10.0;
    double accumulated_distance = 0.0;
    // accumulate lanelet lengths alongside the route, considering possible lane changes
    for (std::size_t i = 1UL; i < route.size(); ++i) {
      // if lane change is required, add the distance traveled during the lane change
      // if lane change is not required, add the current lanelet length
      if (is_lane_change_required(route[i - 1UL], route[i])) {
        const auto current_lanelet_spline = getCenterPointsSpline(route[i - 1UL]);
        const auto next_lanelet_spline = getCenterPointsSpline(route[i]);

        // first, lanelets are matched at the start (s = 0.0) of each lanelet; only if this fails,
        // matching is performed at a larger s value, which should cover the rest of the cases.
        static constexpr double lanelet_starting_s = 0.0;
        const auto lanelet_min_middle_s =
          std::min(current_lanelet_spline->getLength(), next_lanelet_spline->getLength()) * 0.5;

        const auto current_start_matching_s = current_lanelet_spline->getSValue(
          next_lanelet_spline->getPose(lanelet_starting_s), matching_distance);
        const auto next_start_matching_s = next_lanelet_spline->getSValue(
          current_lanelet_spline->getPose(lanelet_starting_s), matching_distance);
        const auto current_middle_matching_s = current_lanelet_spline->getSValue(
          next_lanelet_spline->getPose(lanelet_min_middle_s), matching_distance);
        const auto next_middle_matching_s = next_lanelet_spline->getSValue(
          current_lanelet_spline->getPose(lanelet_min_middle_s), matching_distance);

        if (current_start_matching_s.has_value()) {
          accumulated_distance += current_start_matching_s.value();
        } else if (next_start_matching_s.has_value()) {
          accumulated_distance -= next_start_matching_s.value();
        } else if (current_middle_matching_s.has_value()) {
          accumulated_distance += current_middle_matching_s.value() - lanelet_min_middle_s;
        } else if (next_middle_matching_s.has_value()) {
          accumulated_distance -= next_middle_matching_s.value() - lanelet_min_middle_s;
        } else {
          return std::nullopt;
        }
      } else {
        accumulated_distance += lanelet_map::laneletLength(route[i - 1UL]);
      }
    }
    // subtract the distance already traveled on the first lanelet: from_pose.s
    // and add the distance that needs to be traveled on the last: to_pose.s.
    if (const double distance = accumulated_distance - from_pose.s + to_pose.s; distance >= 0.0) {
      return std::make_optional(distance);
    } else {
      return std::nullopt;
    }
  } else {
    return std::nullopt;
  }
}

auto HdMapUtils::toMapBin() const -> autoware_map_msgs::msg::LaneletMapBin
{
  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *lanelet_map_ptr_;
  auto id_counter = lanelet::utils::getId();
  oa << id_counter;
  std::string tmp_str = ss.str();
  autoware_map_msgs::msg::LaneletMapBin msg;
  msg.data.clear();
  msg.data.resize(tmp_str.size());
  msg.data.assign(tmp_str.begin(), tmp_str.end());
  msg.header.frame_id = "map";
  return msg;
}

auto HdMapUtils::insertMarkerArray(
  visualization_msgs::msg::MarkerArray & a1, const visualization_msgs::msg::MarkerArray & a2) const
  -> void
{
  a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}

auto HdMapUtils::generateMarker() const -> visualization_msgs::msg::MarkerArray
{
  visualization_msgs::msg::MarkerArray markers;
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
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

auto HdMapUtils::overwriteLaneletsCenterline() -> void
{
  for (auto & lanelet_obj : lanelet_map_ptr_->laneletLayer) {
    if (!lanelet_obj.hasCustomCenterline()) {
      const auto fine_center_line = generateFineCenterline(lanelet_obj, 2.0);
      lanelet_obj.setCenterline(fine_center_line);
    }
  }
}

auto HdMapUtils::findNearestIndexPair(
  const std::vector<double> & accumulated_lengths, const double target_length) const
  -> std::pair<std::size_t, std::size_t>
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

auto HdMapUtils::getRightOfWayLaneletIds(const lanelet::Ids & lanelet_ids) const
  -> std::unordered_map<lanelet::Id, lanelet::Ids>
{
  std::unordered_map<lanelet::Id, lanelet::Ids> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    ret.emplace(lanelet_id, getRightOfWayLaneletIds(lanelet_id));
  }
  return ret;
}

auto HdMapUtils::getRightOfWayLaneletIds(const lanelet::Id lanelet_id) const -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & right_of_way :
       lanelet_map_ptr_->laneletLayer.get(lanelet_id).regulatoryElementsAs<lanelet::RightOfWay>()) {
    for (const auto & ll : right_of_way->rightOfWayLanelets()) {
      if (lanelet_id != ll.id()) {
        ids.push_back(ll.id());
      }
    }
  }
  return ids;
}

auto HdMapUtils::getTrafficSignRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids) const
  -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>
{
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const auto traffic_signs = lanelet.regulatoryElementsAs<const lanelet::TrafficSign>();
    for (const auto & traffic_sign : traffic_signs) {
      ret.emplace_back(traffic_sign);
    }
  }
  return ret;
}

auto HdMapUtils::getTrafficSignRegulatoryElements() const
  -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>
{
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> ret;
  for (const auto & lanelet_id : getLaneletIds()) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const auto traffic_signs = lanelet.regulatoryElementsAs<const lanelet::TrafficSign>();
    for (const auto & traffic_sign : traffic_signs) {
      ret.emplace_back(traffic_sign);
    }
  }
  return ret;
}

auto HdMapUtils::getTrafficLightRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids) const
  -> std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>>
{
  std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const auto traffic_lights =
      lanelet.regulatoryElementsAs<const lanelet::autoware::AutowareTrafficLight>();
    for (const auto & traffic_light : traffic_lights) {
      ret.emplace_back(traffic_light);
    }
  }
  return ret;
}

auto HdMapUtils::getStopLines() const -> lanelet::ConstLineStrings3d
{
  lanelet::ConstLineStrings3d ret;
  for (const auto & traffic_sign : getTrafficSignRegulatoryElements()) {
    if (traffic_sign->type() == "stop_sign") {
      for (const auto & stop_line : traffic_sign->refLines()) {
        ret.emplace_back(stop_line);
      }
    }
  }
  return ret;
}

auto HdMapUtils::getStopLinesOnPath(const lanelet::Ids & lanelet_ids) const
  -> lanelet::ConstLineStrings3d
{
  lanelet::ConstLineStrings3d ret;
  for (const auto & traffic_sign : getTrafficSignRegulatoryElementsOnPath(lanelet_ids)) {
    if (traffic_sign->type() == "stop_sign") {
      for (const auto & stop_line : traffic_sign->refLines()) {
        ret.emplace_back(stop_line);
      }
    }
  }
  return ret;
}

auto HdMapUtils::getStopLineIds() const -> lanelet::Ids
{
  lanelet::Ids stop_line_ids;
  for (const auto & ret : getStopLines()) {
    stop_line_ids.push_back(ret.id());
  }
  return stop_line_ids;
}

auto HdMapUtils::getStopLineIdsOnPath(const lanelet::Ids & route_lanelets) const -> lanelet::Ids
{
  lanelet::Ids stop_line_ids;
  for (const auto & ret : getStopLinesOnPath(route_lanelets)) {
    stop_line_ids.push_back(ret.id());
  }
  return stop_line_ids;
}

auto HdMapUtils::getTrafficLights(const lanelet::Id traffic_light_id) const
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>
{
  std::vector<lanelet::AutowareTrafficLightConstPtr> ret;
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  auto autoware_traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (const auto & light : autoware_traffic_lights) {
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

auto HdMapUtils::getTrafficLightStopLineIds(const lanelet::Id traffic_light_id) const
  -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & traffic_light : getTrafficLights(traffic_light_id)) {
    if (traffic_light->stopLine()) {
      ids.push_back(traffic_light->stopLine()->id());
    }
  }
  return ids;
}

auto HdMapUtils::getTrafficLightStopLinesPoints(const lanelet::Id traffic_light_id) const
  -> std::vector<std::vector<geometry_msgs::msg::Point>>
{
  std::vector<std::vector<geometry_msgs::msg::Point>> ret;
  const auto traffic_lights = getTrafficLights(traffic_light_id);
  for (const auto & traffic_light : traffic_lights) {
    ret.emplace_back(std::vector<geometry_msgs::msg::Point>{});
    const auto stop_line = traffic_light->stopLine();
    if (stop_line) {
      auto & current_stop_line = ret.back();
      for (const auto & point : stop_line.value()) {
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

auto HdMapUtils::getStopLinePolygon(const lanelet::Id lanelet_id) const
  -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> points;
  const auto stop_line = lanelet_map_ptr_->lineStringLayer.get(lanelet_id);
  for (const auto & point : stop_line) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    points.emplace_back(p);
  }
  return points;
}

auto HdMapUtils::getTrafficLightIdsOnPath(const lanelet::Ids & route_lanelets) const -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & traffic_light : getTrafficLightRegulatoryElementsOnPath(route_lanelets)) {
    for (auto light_string : traffic_light->lightBulbs()) {
      if (light_string.hasAttribute("traffic_light_id")) {
        if (auto id = light_string.attribute("traffic_light_id").asId(); id) {
          ids.push_back(id.value());
        }
      }
    }
  }
  return ids;
}

auto HdMapUtils::getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets,
  const std::vector<geometry_msgs::msg::Point> & waypoints) const -> std::optional<double>
{
  auto traffic_light_ids = getTrafficLightIdsOnPath(route_lanelets);
  if (traffic_light_ids.empty()) {
    return std::nullopt;
  }
  std::set<double> collision_points;
  for (const auto id : traffic_light_ids) {
    const auto collision_point = getDistanceToTrafficLightStopLine(waypoints, id);
    if (collision_point) {
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto HdMapUtils::getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets,
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  auto traffic_light_ids = getTrafficLightIdsOnPath(route_lanelets);
  if (traffic_light_ids.empty()) {
    return std::nullopt;
  }
  std::set<double> collision_points;
  for (const auto id : traffic_light_ids) {
    const auto collision_point = getDistanceToTrafficLightStopLine(spline, id);
    if (collision_point) {
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto HdMapUtils::getDistanceToTrafficLightStopLine(
  const std::vector<geometry_msgs::msg::Point> & waypoints,
  const lanelet::Id traffic_light_id) const -> std::optional<double>
{
  if (waypoints.empty()) {
    return std::nullopt;
  }
  math::geometry::CatmullRomSpline spline(waypoints);
  const auto stop_lines = getTrafficLightStopLinesPoints(traffic_light_id);
  for (const auto & stop_line : stop_lines) {
    const auto collision_point = spline.getCollisionPointIn2D(stop_line);
    if (collision_point) {
      return collision_point;
    }
  }
  return std::nullopt;
}

auto HdMapUtils::getDistanceToTrafficLightStopLine(
  const math::geometry::CatmullRomSplineInterface & spline,
  const lanelet::Id traffic_light_id) const -> std::optional<double>
{
  if (spline.getLength() <= 0) {
    return std::nullopt;
  }
  const auto stop_lines = getTrafficLightStopLinesPoints(traffic_light_id);
  for (const auto & stop_line : stop_lines) {
    const auto collision_point = spline.getCollisionPointIn2D(stop_line);
    if (collision_point) {
      return collision_point;
    }
  }
  return std::nullopt;
}

auto HdMapUtils::getDistanceToStopLine(
  const lanelet::Ids & route_lanelets,
  const std::vector<geometry_msgs::msg::Point> & waypoints) const -> std::optional<double>
{
  if (waypoints.empty()) {
    return std::nullopt;
  }
  std::set<double> collision_points;
  if (waypoints.empty()) {
    return std::nullopt;
  }
  math::geometry::CatmullRomSpline spline(waypoints);
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
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto HdMapUtils::getDistanceToStopLine(
  const lanelet::Ids & route_lanelets,
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  if (spline.getLength() <= 0) {
    return std::nullopt;
  }
  std::set<double> collision_points;
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
      collision_points.insert(collision_point.value());
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto HdMapUtils::calculateSegmentDistances(const lanelet::ConstLineString3d & line_string) const
  -> std::vector<double>
{
  std::vector<double> segment_distances;
  segment_distances.reserve(line_string.size() - 1);
  for (size_t i = 1; i < line_string.size(); ++i) {
    const auto distance = lanelet::geometry::distance(line_string[i], line_string[i - 1]);
    segment_distances.push_back(distance);
  }
  return segment_distances;
}

auto HdMapUtils::calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string) const
  -> std::vector<double>
{
  const auto segment_distances = calculateSegmentDistances(line_string);

  std::vector<double> accumulated_lengths{0};
  accumulated_lengths.reserve(segment_distances.size() + 1);
  std::partial_sum(
    std::begin(segment_distances), std::end(segment_distances),
    std::back_inserter(accumulated_lengths));
  return accumulated_lengths;
}

auto HdMapUtils::resamplePoints(
  const lanelet::ConstLineString3d & line_string, const std::int32_t num_segments) const
  -> lanelet::BasicPoints3d
{
  // Calculate length
  const auto line_length = lanelet::geometry::length(line_string);

  // Calculate accumulated lengths
  const auto accumulated_lengths = calculateAccumulatedLengths(line_string);

  // Create each segment
  lanelet::BasicPoints3d resampled_points;
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

auto HdMapUtils::generateFineCenterline(
  const lanelet::ConstLanelet & lanelet_obj, const double resolution) const -> lanelet::LineString3d
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

auto HdMapUtils::calcEuclidDist(
  const std::vector<double> & x, const std::vector<double> & y, const std::vector<double> & z) const
  -> std::vector<double>
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

auto HdMapUtils::getLanelets(const lanelet::Ids & lanelet_ids) const -> lanelet::Lanelets
{
  lanelet::Lanelets lanelets;
  for (const auto & id : lanelet_ids) {
    lanelets.emplace_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  return lanelets;
}

auto HdMapUtils::isTrafficLight(const lanelet::Id lanelet_id) const -> bool
{
  using namespace lanelet;

  if (lanelet_map_ptr_->lineStringLayer.exists(lanelet_id)) {
    if (auto && linestring = lanelet_map_ptr_->lineStringLayer.get(lanelet_id);
        linestring.hasAttribute(AttributeName::Type)) {
      return linestring.attribute(AttributeName::Type).value() == "traffic_light";
    }
  }

  return false;
}

auto HdMapUtils::isTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) const -> bool
{
  return lanelet_map_ptr_->regulatoryElementLayer.exists(lanelet_id) and
         std::dynamic_pointer_cast<lanelet::TrafficLight>(
           lanelet_map_ptr_->regulatoryElementLayer.get(lanelet_id));
}

auto HdMapUtils::getTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) const
  -> lanelet::TrafficLight::Ptr
{
  assert(isTrafficLightRegulatoryElement(lanelet_id));
  return std::dynamic_pointer_cast<lanelet::TrafficLight>(
    lanelet_map_ptr_->regulatoryElementLayer.get(lanelet_id));
}

auto HdMapUtils::getTrafficLightRegulatoryElementIDsFromTrafficLight(
  const lanelet::Id traffic_light_way_id) const -> lanelet::Ids
{
  assert(isTrafficLight(traffic_light_way_id));
  lanelet::Ids traffic_light_regulatory_element_ids;
  for (const auto & regulatory_element : lanelet_map_ptr_->regulatoryElementLayer) {
    if (regulatory_element->attribute(lanelet::AttributeName::Subtype).value() == "traffic_light") {
      for (const auto & ref_member :
           regulatory_element->getParameters<lanelet::ConstLineString3d>("refers")) {
        if (ref_member.id() == traffic_light_way_id) {
          traffic_light_regulatory_element_ids.push_back(regulatory_element->id());
        }
      }
    }
  }
  return traffic_light_regulatory_element_ids;
}

auto HdMapUtils::toPolygon(const lanelet::ConstLineString3d & line_string) const
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

HdMapUtils::RoutingGraphs::RoutingGraphs(const lanelet::LaneletMapPtr & lanelet_map)
{
  vehicle.rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  vehicle.graph = lanelet::routing::RoutingGraph::build(*lanelet_map, *vehicle.rules);
  vehicle_with_road_shoulder.rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    Locations::RoadShoulderPassableGermany, lanelet::Participants::Vehicle);
  vehicle_with_road_shoulder.graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map, *vehicle_with_road_shoulder.rules);
  pedestrian.rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  pedestrian.graph = lanelet::routing::RoutingGraph::build(*lanelet_map, *pedestrian.rules);
}

lanelet::routing::RoutingGraphPtr HdMapUtils::RoutingGraphs::routing_graph(
  const traffic_simulator::RoutingGraphType type) const
{
  switch (type) {
    case traffic_simulator::RoutingGraphType::VEHICLE:
      return vehicle.graph;
    case traffic_simulator::RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER:
      return vehicle_with_road_shoulder.graph;
    case traffic_simulator::RoutingGraphType::PEDESTRIAN:
      return pedestrian.graph;
    default:
      std::stringstream what;
      what << "Invalid routing graph type: " << static_cast<int>(type);
      throw common::Error(what.str());
  }
}

lanelet::traffic_rules::TrafficRulesPtr HdMapUtils::RoutingGraphs::traffic_rule(
  const traffic_simulator::RoutingGraphType type) const
{
  switch (type) {
    case traffic_simulator::RoutingGraphType::VEHICLE:
      return vehicle.rules;
    case traffic_simulator::RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER:
      return vehicle_with_road_shoulder.rules;
    case traffic_simulator::RoutingGraphType::PEDESTRIAN:
      return pedestrian.rules;
    default:
      std::stringstream what;
      what << "Invalid routing graph type: " << static_cast<int>(type);
      throw common::Error(what.str());
  }
}

RouteCache & HdMapUtils::RoutingGraphs::route_cache(const traffic_simulator::RoutingGraphType type)
{
  switch (type) {
    case traffic_simulator::RoutingGraphType::VEHICLE:
      return vehicle.route_cache;
    case traffic_simulator::RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER:
      return vehicle_with_road_shoulder.route_cache;
    case traffic_simulator::RoutingGraphType::PEDESTRIAN:
      return pedestrian.route_cache;
    default:
      std::stringstream what;
      what << "Invalid routing graph type: " << static_cast<int>(type);
      throw common::Error(what.str());
  }
}

auto HdMapUtils::RoutingGraphs::getRoute(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id,
  lanelet::LaneletMapPtr lanelet_map_ptr,
  const traffic_simulator::RoutingConfiguration & routing_configuration) -> lanelet::Ids
{
  auto & cache = route_cache(routing_configuration.routing_graph_type);
  if (cache.exists(from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change)) {
    return cache.getRoute(from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change);
  }
  lanelet::Ids ids;
  const auto lanelet = lanelet_map_ptr->laneletLayer.get(from_lanelet_id);
  const auto to_lanelet = lanelet_map_ptr->laneletLayer.get(to_lanelet_id);
  lanelet::Optional<lanelet::routing::Route> route =
    routing_graph(routing_configuration.routing_graph_type)
      ->getRoute(lanelet, to_lanelet, 0, routing_configuration.allow_lane_change);
  if (!route) {
    cache.appendData(from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change, ids);
    return ids;
  }
  lanelet::routing::LaneletPath shortest_path = route->shortestPath();
  if (shortest_path.empty()) {
    cache.appendData(from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change, ids);
    return ids;
  }
  for (auto lane_itr = shortest_path.begin(); lane_itr != shortest_path.end(); lane_itr++) {
    ids.push_back(lane_itr->id());
  }
  cache.appendData(from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change, ids);
  return ids;
}

}  // namespace hdmap_utils
