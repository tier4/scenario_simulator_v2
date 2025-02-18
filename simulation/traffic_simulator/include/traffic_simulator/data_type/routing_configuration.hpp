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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__ROUTING_CONFIGURATIONS_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__ROUTING_CONFIGURATIONS_HPP_

#include <iostream>
#include <traffic_simulator/data_type/routing_graph_type.hpp>

namespace traffic_simulator
{
struct RoutingConfiguration
{
  RoutingConfiguration() = default;
  explicit RoutingConfiguration(const bool allow_lane_change)
  : allow_lane_change(allow_lane_change){};

  bool allow_lane_change = false;
  traffic_simulator::RoutingGraphType routing_graph_type =
    traffic_simulator::RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER;

  bool operator==(const RoutingConfiguration & routing_configuration) const
  {
    return allow_lane_change == routing_configuration.allow_lane_change &&
           routing_graph_type == routing_configuration.routing_graph_type;
  }

  friend std::ostream & operator<<(std::ostream & os, const RoutingConfiguration & rc)
  {
    os << "{ allow_lane_change: " << (rc.allow_lane_change ? "true" : "false")
       << ", routing_graph_type: " << to_string(rc.routing_graph_type) << " }";
    return os;
  }
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__ROUTING_CONFIGURATIONS_HPP_
