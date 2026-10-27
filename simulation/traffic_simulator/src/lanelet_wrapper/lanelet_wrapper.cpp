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

#include <traffic_simulator/lanelet_wrapper/lanelet_loader.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>
#include <traffic_simulator/lanelet_wrapper/traffic_rules.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
CenterPointsCache::CenterPointsCache(const lanelet::LaneletMapPtr & lanelet_map)
{
  data_.reserve(lanelet_map->laneletLayer.size());
  splines_.reserve(lanelet_map->laneletLayer.size());
  for (const auto & lanelet : lanelet_map->laneletLayer) {
    const auto lanelet_id = lanelet.id();
    const auto & points =
      data_.emplace(lanelet_id, centerPoints(lanelet_id, lanelet_map)).first->second;
    splines_.emplace(lanelet_id, std::make_shared<Spline>(points));
  }
}

LaneletLengthCache::LaneletLengthCache(const lanelet::LaneletMapPtr & lanelet_map)
{
  data_.reserve(lanelet_map->laneletLayer.size());
  for (const auto & lanelet : lanelet_map->laneletLayer) {
    data_.emplace(lanelet.id(), lanelet::utils::getLaneletLength2d(lanelet));
  }
}

auto TopologyCache::getNextLaneletIds(
  const lanelet::Id lanelet_id, const RoutingGraphType type) const -> const lanelet::Ids &
{
  return next_lanelet_ids_cache_.at(static_cast<std::size_t>(type)).at(lanelet_id);
}

auto TopologyCache::getPreviousLaneletIds(
  const lanelet::Id lanelet_id, const RoutingGraphType type) const -> const lanelet::Ids &
{
  return previous_lanelet_ids_cache_.at(static_cast<std::size_t>(type)).at(lanelet_id);
}

auto TopologyCache::getRightOfWayLaneletIds(const lanelet::Id lanelet_id) const
  -> const lanelet::Ids &
{
  return right_of_way_lanelet_ids_cache_.at(lanelet_id);
}

auto TopologyCache::getConflictingCrosswalkIds(const lanelet::Id lanelet_id) const
  -> const lanelet::Ids &
{
  return conflicting_crosswalk_ids_cache_.at(lanelet_id);
}

auto TopologyCache::getConflictingLaneIds(
  const lanelet::Id lanelet_id, const RoutingGraphType type) const -> const lanelet::Ids &
{
  return conflicting_lanelet_ids_cache_.at(static_cast<std::size_t>(type)).at(lanelet_id);
}

TopologyCache::TopologyCache(
  const lanelet::LaneletMapPtr & lanelet_map,
  const std::array<lanelet::routing::RoutingGraphConstPtr, 3> & routing_graphs)
{
  lanelet::routing::RoutingGraphContainer graphs_container(
    {routing_graphs[static_cast<std::size_t>(RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER)],
     routing_graphs[static_cast<std::size_t>(RoutingGraphType::PEDESTRIAN)]});
  constexpr std::size_t routing_graph_id{1};
  constexpr double height_clearance{4};

  for (const auto & lanelet : lanelet_map->laneletLayer) {
    const auto lanelet_id = lanelet.id();

    for (std::size_t i = 0; i < routing_graphs.size(); ++i) {
      const auto & graph = routing_graphs[i];

      auto & next_lanelet_ids = next_lanelet_ids_cache_[i][lanelet_id];
      const auto & following_lanelets = graph->following(lanelet);
      next_lanelet_ids.reserve(following_lanelets.size());
      for (const auto & following_lanelet : following_lanelets) {
        next_lanelet_ids.push_back(following_lanelet.id());
      }

      auto & previous_lanelet_ids = previous_lanelet_ids_cache_[i][lanelet_id];
      const auto & previous_lanelets = graph->previous(lanelet);
      previous_lanelet_ids.reserve(previous_lanelets.size());
      for (const auto & previous_lanelet : previous_lanelets) {
        previous_lanelet_ids.push_back(previous_lanelet.id());
      }

      auto & conflicting_lanes_ids = conflicting_lanelet_ids_cache_[i][lanelet_id];
      const auto & conflicting_lanelets = lanelet::utils::getConflictingLanelets(graph, lanelet);
      conflicting_lanes_ids.reserve(conflicting_lanelets.size());
      for (const auto & conflicting_lanelet : conflicting_lanelets) {
        conflicting_lanes_ids.push_back(conflicting_lanelet.id());
      }
    }

    auto & right_of_way_lanelets_ids = right_of_way_lanelet_ids_cache_[lanelet_id];
    const auto & right_of_ways = lanelet.regulatoryElementsAs<lanelet::RightOfWay>();
    for (const auto & right_of_way : right_of_ways) {
      for (const auto & right_of_way_lanelet : right_of_way->rightOfWayLanelets()) {
        if (right_of_way_lanelet.id() != lanelet_id) {
          right_of_way_lanelets_ids.push_back(right_of_way_lanelet.id());
        }
      }
    }

    auto & conflicting_crosswalk_ids = conflicting_crosswalk_ids_cache_[lanelet_id];
    /// @note it is not clear if the distinction for crosswalks only is implemented here
    const auto & conflicting_crosswalks =
      graphs_container.conflictingInGraph(lanelet, routing_graph_id, height_clearance);
    conflicting_crosswalk_ids.reserve(conflicting_crosswalks.size());
    for (const auto & conflicting_crosswalk : conflicting_crosswalks) {
      conflicting_crosswalk_ids.push_back(conflicting_crosswalk.id());
    }
  }
}

auto LaneletWrapper::activate(const std::string & lanelet_map_path) -> void
{
  lanelet_map_path_ = lanelet_map_path;
  if (instance) {
    std::lock_guard lock(mutex_);
    instance.reset();
  }
}

auto LaneletWrapper::map() -> lanelet::LaneletMapPtr { return getInstance().lanelet_map_ptr_; }

auto LaneletWrapper::routingGraph(const RoutingGraphType type)
  -> lanelet::routing::RoutingGraphConstPtr
{
  switch (type) {
    case RoutingGraphType::VEHICLE:
      return getInstance().vehicle_.graph;
    case RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER:
      return getInstance().vehicle_with_road_shoulder_.graph;
    case RoutingGraphType::PEDESTRIAN:
      return getInstance().pedestrian_.graph;
    default:
      std::stringstream what;
      what << "Invalid routing graph type: " << static_cast<int>(type);
      throw common::Error(what.str());
  }
}

auto LaneletWrapper::trafficRules(const RoutingGraphType type)
  -> lanelet::traffic_rules::TrafficRulesPtr
{
  switch (type) {
    case RoutingGraphType::VEHICLE:
      return getInstance().vehicle_.rules;
    case RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER:
      return getInstance().vehicle_with_road_shoulder_.rules;
    case RoutingGraphType::PEDESTRIAN:
      return getInstance().pedestrian_.rules;
    default:
      std::stringstream what;
      what << "Invalid routing graph type: " << static_cast<int>(type);
      throw common::Error(what.str());
  }
}

auto LaneletWrapper::routeCache(const RoutingGraphType type) -> RouteCache &
{
  switch (type) {
    case RoutingGraphType::VEHICLE:
      return getInstance().vehicle_.route_cache;
    case RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER:
      return getInstance().vehicle_with_road_shoulder_.route_cache;
    case RoutingGraphType::PEDESTRIAN:
      return getInstance().pedestrian_.route_cache;
    default:
      std::stringstream what;
      what << "Invalid routing graph type: " << static_cast<int>(type);
      throw common::Error(what.str());
  }
}

auto LaneletWrapper::centerPointsCache() -> const CenterPointsCache &
{
  return getInstance().center_points_cache_;
}

auto LaneletWrapper::laneletLengthCache() -> const LaneletLengthCache &
{
  return getInstance().lanelet_length_cache_;
}

auto LaneletWrapper::topologyCache() -> const TopologyCache &
{
  return getInstance().topology_cache_;
}

LaneletWrapper::LaneletWrapper(const std::filesystem::path & lanelet_map_path)
: lanelet_map_ptr_(LaneletLoader::load(lanelet_map_path)),
  vehicle_(lanelet_map_ptr_, lanelet::Locations::Germany, lanelet::Participants::Vehicle),
  vehicle_with_road_shoulder_(
    lanelet_map_ptr_, Locations::RoadShoulderPassableGermany, lanelet::Participants::Vehicle),
  pedestrian_(lanelet_map_ptr_, lanelet::Locations::Germany, lanelet::Participants::Pedestrian),
  center_points_cache_(lanelet_map_ptr_),
  lanelet_length_cache_(lanelet_map_ptr_),
  topology_cache_(
    lanelet_map_ptr_, {vehicle_.graph, pedestrian_.graph, vehicle_with_road_shoulder_.graph})
{
}

LaneletWrapper & LaneletWrapper::getInstance()
{
  std::lock_guard lock(mutex_);
  if (!instance) {
    if (!lanelet_map_path_.empty()) {
      /// @note `new` is intentionally used here instead of `make_unique` since the LaneletWrapper constructor is private
      instance.reset(new LaneletWrapper(lanelet_map_path_));
    } else {
      THROW_SIMULATION_ERROR(
        "LaneletWrapper::lanelet_map_path_ is empty! Please call lanelet_map::activate() first.");
    }
  }
  return *instance;
}
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
