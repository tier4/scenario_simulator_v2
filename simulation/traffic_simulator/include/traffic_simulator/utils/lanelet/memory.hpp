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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_MEMORY_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_MEMORY_HPP_

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <filesystem>
#include <traffic_simulator/utils/lanelet/cache.hpp>

namespace traffic_simulator
{
namespace lanelet2
{
class Memory
{
public:
  static auto routeCache() -> RouteCache &;
  static auto centerPointsCache() -> CenterPointsCache &;
  static auto laneletLengthCache() -> LaneletLengthCache &;

  static auto activate(const std::string & lanelet_map_path) -> void;
  static auto laneletMap() -> const lanelet::LaneletMapPtr &;
  static auto shoulderLanelets() -> const lanelet::ConstLanelets &;
  static auto vehicleRoutingGraph() -> const lanelet::routing::RoutingGraphConstPtr &;
  static auto pedestrianRoutingGraph() -> const lanelet::routing::RoutingGraphConstPtr &;
  static auto trafficRulesVehicle() -> const lanelet::traffic_rules::TrafficRulesPtr &;
  static auto trafficRulesPedestrian() -> const lanelet::traffic_rules::TrafficRulesPtr &;

private:
  Memory(const std::filesystem::path & lanelet2_map_path);
  static Memory & getInstance();

  auto overwriteLaneletsCenterline() -> void;

  auto generateFineCenterline(const lanelet::ConstLanelet & lanelet_obj, const double resolution)
    -> lanelet::LineString3d;

  auto resamplePoints(
    const lanelet::ConstLineString3d & line_string, const std::int32_t num_segments)
    -> lanelet::BasicPoints3d;

  auto findNearestIndexPair(
    const std::vector<double> & accumulated_lengths, const double target_length)
    -> std::pair<std::size_t, std::size_t>;

  auto calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string)
    -> std::vector<double>;

  auto calculateSegmentDistances(const lanelet::ConstLineString3d & line_string)
    -> std::vector<double>;

  inline static std::unique_ptr<Memory> instance{nullptr};
  inline static std::string lanelet_map_path_{""};
  inline static std::mutex mutex_;

  mutable RouteCache route_cache_;
  mutable CenterPointsCache center_points_cache_;
  mutable LaneletLengthCache lanelet_length_cache_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstLanelets shoulder_lanelets_;
  lanelet::routing::RoutingGraphConstPtr vehicle_routing_graph_ptr_;
  lanelet::routing::RoutingGraphConstPtr pedestrian_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_vehicle_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_pedestrian_ptr_;
};
}  // namespace lanelet2
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_MEMORY_HPP_