// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__WAYPOINT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__WAYPOINT_HPP_

#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/syntax/route_strategy.hpp>

#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Waypoint ---------------------------------------------------------------
 *
 *  <xsd:complexType name="Waypoint">
 *    <xsd:sequence>
 *      <xsd:element name="Position" type="Position"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="routeStrategy" type="RouteStrategy" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Waypoint
{
  const RouteStrategy route_strategy;

  const Position position;

  template<typename Node, typename Scope>
  explicit Waypoint(const Node & node, Scope & outer_scope)
  : route_strategy(readAttribute<RouteStrategy>("routeStrategy", node, outer_scope)),
    position(readElement<Position>("Position", node, outer_scope))
  {}

  explicit operator geometry_msgs::msg::Pose() const
  {
    return static_cast<geometry_msgs::msg::Pose>(position);
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__WAYPOINT_HPP_
