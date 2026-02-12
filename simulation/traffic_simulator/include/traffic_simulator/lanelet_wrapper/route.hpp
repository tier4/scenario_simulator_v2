// Copyright 2015 Tier IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__LANELET_WRAPPER_ROUTE_HPP_
#define TRAFFIC_SIMULATOR__LANELET_WRAPPER_ROUTE_HPP_

#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace route
{
auto isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route_lanelets_ids) -> bool;

auto speedLimit(
  const lanelet::Ids & lanelet_ids,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> double;

auto routeFromGraph(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id,
  const RoutingConfiguration & routing_configuration = RoutingConfiguration()) -> lanelet::Ids;

auto followingLanelets(
  const lanelet::Id lanelet_id, const lanelet::Ids & candidate_lanelet_ids,
  const double distance = 100, const bool include_self = true,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> lanelet::Ids;

auto followingLanelets(
  const lanelet::Id lanelet_id, const double distance = 100, const bool include_self = true,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> lanelet::Ids;

auto previousLanelets(
  const lanelet::Id current_lanelet_id, const double backward_horizon = 100,
  const RoutingGraphType type = RoutingConfiguration().routing_graph_type) -> lanelet::Ids;
}  // namespace route
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__LANELET_WRAPPER_ROUTE_HPP_
