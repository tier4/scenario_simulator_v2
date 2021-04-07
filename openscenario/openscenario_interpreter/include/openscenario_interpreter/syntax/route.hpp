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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ROUTE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ROUTE_HPP_

#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/waypoint.hpp>
#include <string>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Route ------------------------------------------------------------------
 *
 *  <xsd:complexType name="Route">
 *    <xsd:sequence>
 *      <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *      <xsd:element name="Waypoint" minOccurs="2" maxOccurs="unbounded" type="Waypoint"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="closed" type="Boolean" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Route
{
  const String name;

  const Boolean closed;

  Scope inner_scope;

  std::vector<Waypoint> waypoints;

  template <typename Node>
  explicit Route(const Node & node, Scope & outer_scope)
  : name(readAttribute<String>("name", node, outer_scope)),
    closed(readAttribute<Boolean>("closed", node, outer_scope, Boolean())),
    inner_scope(outer_scope)
  {
    callWithElements(node, "ParameterDeclarations", 0, 1, [&](auto && node) {
      return ParameterDeclarations(node, inner_scope);
    });

    callWithElements(node, "Waypoint", 2, unbounded, [&](auto && node) {
      return waypoints.emplace_back(node, inner_scope);
    });
  }

  explicit operator std::vector<openscenario_msgs::msg::LaneletPose>() const
  {
    std::vector<openscenario_msgs::msg::LaneletPose> lanelet_poses{};

    for (const auto & waypoint : waypoints) {
      lanelet_poses.emplace_back(static_cast<openscenario_msgs::msg::LaneletPose>(waypoint));
    }

    return lanelet_poses;
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ROUTE_HPP_
