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
    case RoutingGraphType::vehicle:
      return getInstance().vehicle_.graph;
    case RoutingGraphType::vehicle_with_road_shoulder:
      return getInstance().vehicle_with_road_shoulder_.graph;
    case RoutingGraphType::pedestrian:
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
    case RoutingGraphType::vehicle:
      return getInstance().vehicle_.rules;
    case RoutingGraphType::vehicle_with_road_shoulder:
      return getInstance().vehicle_with_road_shoulder_.rules;
    case RoutingGraphType::pedestrian:
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
    case RoutingGraphType::vehicle:
      return getInstance().vehicle_.route_cache;
    case RoutingGraphType::vehicle_with_road_shoulder:
      return getInstance().vehicle_with_road_shoulder_.route_cache;
    case RoutingGraphType::pedestrian:
      return getInstance().pedestrian_.route_cache;
    default:
      std::stringstream what;
      what << "Invalid routing graph type: " << static_cast<int>(type);
      throw common::Error(what.str());
  }
}

auto LaneletWrapper::centerPointsCache() -> CenterPointsCache &
{
  return getInstance().center_points_cache_;
}

auto LaneletWrapper::laneletLengthCache() -> LaneletLengthCache &
{
  return getInstance().lanelet_length_cache_;
}

LaneletWrapper::LaneletWrapper(const std::filesystem::path & lanelet_map_path)
: lanelet_map_ptr_(LaneletLoader::load(lanelet_map_path)),
  vehicle_(lanelet_map_ptr_, lanelet::Locations::Germany, lanelet::Participants::Vehicle),
  vehicle_with_road_shoulder_(
    lanelet_map_ptr_, Locations::RoadShoulderPassableGermany, lanelet::Participants::Vehicle),
  pedestrian_(lanelet_map_ptr_, lanelet::Locations::Germany, lanelet::Participants::Pedestrian)
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
