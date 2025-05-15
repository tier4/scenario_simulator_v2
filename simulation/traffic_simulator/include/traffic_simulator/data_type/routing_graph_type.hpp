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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__ROUTING_GRAPH_TYPE_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__ROUTING_GRAPH_TYPE_HPP_

#include <iostream>

namespace traffic_simulator
{
inline namespace routing_graph_type
{
enum class RoutingGraphType : std::uint8_t { vehicle, pedestrian, vehicle_with_road_shoulder };

char const * to_string(const RoutingGraphType &);
}  // namespace routing_graph_type
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__ROUTING_GRAPH_TYPE_HPP_
