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

#include <traffic_simulator/data_type/routing_graph_type.hpp>

namespace traffic_simulator
{
namespace routing_graph_type
{
char const * to_string(const RoutingGraphType & type)
{
  switch (type) {
    case RoutingGraphType::vehicle:
      return "vehicle";
    case RoutingGraphType::pedestrian:
      return "pedestrian";
    case RoutingGraphType::vehicle_with_road_shoulder:
      return "vehicle_with_road_shoulder";
    default:
      return "unknown";
  }
}
}  // namespace routing_graph_type
}  // namespace traffic_simulator
