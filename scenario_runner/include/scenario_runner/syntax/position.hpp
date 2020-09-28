// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__SYNTAX__POSITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__POSITION_HPP_

#include <scenario_runner/syntax/lane_position.hpp>
#include <scenario_runner/syntax/world_position.hpp>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== Position =============================================================
 *
 * <xsd:complexType name="Position">
 *   <xsd:choice>
 *     <xsd:element name="WorldPosition" type="WorldPosition"/>
 *     <xsd:element name="RelativeWorldPosition" type="RelativeWorldPosition"/>
 *     <xsd:element name="RelativeObjectPosition" type="RelativeObjectPosition"/>
 *     <xsd:element name="RoadPosition" type="RoadPosition"/>
 *     <xsd:element name="RelativeRoadPosition" type="RelativeRoadPosition"/>
 *     <xsd:element name="LanePosition" type="LanePosition"/>
 *     <xsd:element name="RelativeLanePosition" type="RelativeLanePosition"/>
 *     <xsd:element name="RoutePosition" type="RoutePosition"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Position
  : public Object
{
  template<typename Node, typename Scope>
  explicit Position(const Node & node, Scope & scope)
  {
    callWithElements(node, "WorldPosition", 0, 1, [&](auto && node)
      {
        return rebind<WorldPosition>(node, scope);
      });

    callWithElements(node, "RelativeWorldPosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
    callWithElements(node, "RelativeObjectPosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
    callWithElements(node, "RoadPosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
    callWithElements(node, "RelativeRoadPosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "LanePosition", 0, 1, [&](auto && node)
      {
        return rebind<LanePosition>(node, scope);
      });

    callWithElements(node, "RelativeLanePosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
    callWithElements(node, "RoutePosition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__POSITION_HPP_
