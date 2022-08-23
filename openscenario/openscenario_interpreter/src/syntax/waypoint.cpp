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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/waypoint.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Waypoint::Waypoint(const pugi::xml_node & node, Scope & scope)
: route_strategy(readAttribute<RouteStrategy>("routeStrategy", node, scope)),
  position(readElement<Position>("Position", node, scope))
{
}

Waypoint::operator traffic_simulator_msgs::msg::LaneletPose() const
{
  return apply<traffic_simulator_msgs::msg::LaneletPose>(
    [](auto && position) {
      return static_cast<traffic_simulator_msgs::msg::LaneletPose>(position);
    },
    position);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
