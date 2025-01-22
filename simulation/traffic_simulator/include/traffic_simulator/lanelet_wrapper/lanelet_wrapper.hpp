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

#ifndef TRAFFIC_SIMULATOR__LANELET_WRAPPER_HPP_
#define TRAFFIC_SIMULATOR__LANELET_WRAPPER_HPP_

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <filesystem>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <mutex>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/data_type/routing_configuration.hpp>
#include <traffic_simulator/data_type/routing_graph_type.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace std
{
template <>
struct hash<std::tuple<lanelet::Id, lanelet::Id, bool>>
{
public:
  size_t operator()(const std::tuple<lanelet::Id, lanelet::Id, bool> & data) const
  {
    std::hash<lanelet::Id> lanelet_id_hash;
    size_t seed = 0;
    /// @note hash combine like boost library 2^32 / phi = 0x9e3779b9
    seed ^= lanelet_id_hash(std::get<0>(data)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= lanelet_id_hash(std::get<1>(data)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<bool>{}(std::get<2>(data)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};
}  // namespace std

namespace traffic_simulator
{
namespace lanelet_wrapper
{
using BoundingBox = traffic_simulator_msgs::msg::BoundingBox;
using EntityType = traffic_simulator_msgs::msg::EntityType;
using LaneletPose = traffic_simulator_msgs::msg::LaneletPose;
using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Spline = math::geometry::CatmullRomSpline;
using Vector3 = geometry_msgs::msg::Vector3;

class RouteCache
{
public:
  auto getRoute(
    const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id,
    const lanelet::LaneletMapPtr & lanelet_map, const RoutingConfiguration & routing_configuration,
    const lanelet::routing::RoutingGraphConstPtr & routing_graph) -> lanelet::Ids
  {
    if (!exists(from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change)) {
      /// @note to use default routing costs: distance along lanelets
      constexpr int routing_cost_id = 0;
      const auto & from_lanelet = lanelet_map->laneletLayer.get(from_lanelet_id);
      const auto & to_lanelet = lanelet_map->laneletLayer.get(to_lanelet_id);
      if (const auto route = routing_graph->getRoute(
            from_lanelet, to_lanelet, routing_cost_id, routing_configuration.allow_lane_change);
          !route || route->shortestPath().empty()) {
        appendData(
          from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change, lanelet::Ids());
      } else {
        lanelet::Ids shortest_path_ids;
        for (const auto & lanelet : route->shortestPath()) {
          shortest_path_ids.push_back(lanelet.id());
        }
        appendData(
          from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change,
          shortest_path_ids);
      }
    }
    std::lock_guard lock(mutex_);
    return data_.at({from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change});
  }

  auto getRoute(const lanelet::Id from, const lanelet::Id to, const bool allow_lane_change)
    -> decltype(auto)
  {
    if (!exists(from, to, allow_lane_change)) {
      THROW_SIMULATION_ERROR(
        "route from : ", from, " to : ", to, (allow_lane_change ? " with" : " without"),
        " lane change does not exists on route cache.");
    } else {
      std::lock_guard lock(mutex_);
      return data_.at({from, to, allow_lane_change});
    }
  }

private:
  auto exists(const lanelet::Id from, const lanelet::Id to, const bool allow_lane_change) -> bool
  {
    std::lock_guard lock(mutex_);
    std::tuple<lanelet::Id, lanelet::Id, bool> key = {from, to, allow_lane_change};
    return data_.find(key) != data_.end();
  }

  auto appendData(
    const lanelet::Id from, const lanelet::Id to, const bool allow_lane_change,
    const lanelet::Ids & route) -> void
  {
    std::lock_guard lock(mutex_);
    data_[{from, to, allow_lane_change}] = route;
  }

  std::unordered_map<std::tuple<lanelet::Id, lanelet::Id, bool>, lanelet::Ids> data_;
  std::mutex mutex_;
};

class CenterPointsCache
{
public:
  auto centerPoints(lanelet::Id lanelet_id) -> decltype(auto)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("center point of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard lock(mutex_);
    return data_.at(lanelet_id);
  }

  auto centerPointsSpline(lanelet::Id lanelet_id) -> decltype(auto)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("center point of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard lock(mutex_);
    return splines_.at(lanelet_id);
  }

  auto getCenterPoints(const lanelet::Id lanelet_id, const lanelet::LaneletMapPtr & lanelet_map)
    -> std::vector<Point>
  {
    if (!exists(lanelet_id)) {
      appendData(lanelet_id, centerPoints(lanelet_id, lanelet_map));
    }
    std::lock_guard lock(mutex_);
    return data_.at(lanelet_id);
  }

  auto getCenterPointsSpline(
    const lanelet::Id lanelet_id, const lanelet::LaneletMapPtr & lanelet_map)
    -> std::shared_ptr<Spline>
  {
    if (!exists(lanelet_id)) {
      appendData(lanelet_id, centerPoints(lanelet_id, lanelet_map));
    }
    std::lock_guard lock(mutex_);
    return splines_.at(lanelet_id);
  }

private:
  auto exists(const lanelet::Id lanelet_id) -> bool
  {
    std::lock_guard lock(mutex_);
    return data_.find(lanelet_id) != data_.end();
  }

  auto appendData(const lanelet::Id lanelet_id, const std::vector<Point> & route) -> void
  {
    std::lock_guard lock(mutex_);
    data_[lanelet_id] = route;
    splines_[lanelet_id] = std::make_shared<Spline>(route);
  }

  auto centerPoints(const lanelet::Id lanelet_id, const lanelet::LaneletMapPtr & lanelet_map) const
    -> std::vector<Point>
  {
    std::vector<Point> center_points;
    for (const auto & point : lanelet_map->laneletLayer.get(lanelet_id).centerline()) {
      center_points.push_back(geometry_msgs::build<Point>().x(point.x()).y(point.y()).z(point.z()));
    }
    if (center_points.size() == 2) {
      const auto p0 = center_points[0];
      const auto p2 = center_points[1];
      const auto p1 = geometry_msgs::build<Point>()
                        .x((p0.x + p2.x) * 0.5)
                        .y((p0.y + p2.y) * 0.5)
                        .z((p0.z + p2.z) * 0.5);
      center_points.clear();
      center_points.push_back(p0);
      center_points.push_back(p1);
      center_points.push_back(p2);
    }
    return center_points;
  }

  std::unordered_map<lanelet::Id, std::vector<Point>> data_;
  std::unordered_map<lanelet::Id, std::shared_ptr<Spline>> splines_;
  std::mutex mutex_;
};

class LaneletLengthCache
{
public:
  auto getLength(lanelet::Id lanelet_id)
  {
    if (!exists(lanelet_id)) {
      THROW_SIMULATION_ERROR("length of : ", lanelet_id, " does not exists on route cache.");
    }
    std::lock_guard lock(mutex_);
    return data_.at(lanelet_id);
  }

  auto getLength(const lanelet::Id lanelet_id, const lanelet::LaneletMapPtr & lanelet_map) -> double
  {
    if (!exists(lanelet_id)) {
      appendData(
        lanelet_id, lanelet::utils::getLaneletLength2d(lanelet_map->laneletLayer.get(lanelet_id)));
    }
    std::lock_guard lock(mutex_);
    return data_.at(lanelet_id);
  }

private:
  auto exists(const lanelet::Id lanelet_id) -> bool
  {
    std::lock_guard lock(mutex_);
    return data_.find(lanelet_id) != data_.end();
  }

  auto appendData(const lanelet::Id lanelet_id, double length) -> void
  {
    std::lock_guard lock(mutex_);
    data_[lanelet_id] = length;
  }

  std::unordered_map<lanelet::Id, double> data_;
  std::mutex mutex_;
};

struct TrafficRulesWithRoutingGraph
{
  lanelet::traffic_rules::TrafficRulesPtr rules;
  lanelet::routing::RoutingGraphConstPtr graph;
  mutable RouteCache route_cache;

  TrafficRulesWithRoutingGraph(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const std::string & locations,
    const std::string & participants)
  {
    rules = lanelet::traffic_rules::TrafficRulesFactory::create(locations, participants);
    graph = lanelet::routing::RoutingGraph::build(*lanelet_map_ptr, *rules);
  }
};

class LaneletWrapper
{
public:
  static auto activate(const std::string & lanelet_map_path) -> void;

  [[nodiscard]] static auto map() -> lanelet::LaneletMapPtr;
  [[nodiscard]] static auto routingGraph(const RoutingGraphType type)
    -> lanelet::routing::RoutingGraphConstPtr;
  [[nodiscard]] static auto trafficRules(const RoutingGraphType type)
    -> lanelet::traffic_rules::TrafficRulesPtr;

  [[nodiscard]] static auto routeCache(const RoutingGraphType type) -> RouteCache &;
  [[nodiscard]] static auto centerPointsCache() -> CenterPointsCache &;
  [[nodiscard]] static auto laneletLengthCache() -> LaneletLengthCache &;

private:
  explicit LaneletWrapper(const std::filesystem::path & lanelet_map_path);
  static LaneletWrapper & getInstance();

  inline static std::unique_ptr<LaneletWrapper> instance{nullptr};
  inline static std::string lanelet_map_path_{""};
  inline static std::mutex mutex_;

  const lanelet::LaneletMapPtr lanelet_map_ptr_;

  const TrafficRulesWithRoutingGraph vehicle_;
  const TrafficRulesWithRoutingGraph vehicle_with_road_shoulder_;
  const TrafficRulesWithRoutingGraph pedestrian_;

  mutable CenterPointsCache center_points_cache_;
  mutable LaneletLengthCache lanelet_length_cache_;
};
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__LANELET_WRAPPER_HPP_
