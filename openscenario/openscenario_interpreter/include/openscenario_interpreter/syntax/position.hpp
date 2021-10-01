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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_

#include <openscenario_interpreter/syntax/lane_position.hpp>
#include <openscenario_interpreter/syntax/relative_world_position.hpp>
#include <openscenario_interpreter/syntax/world_position.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Position ---------------------------------------------------------------
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
 * -------------------------------------------------------------------------- */
struct Position : public Element
{
  template <typename XML, typename... Ts>
  explicit Position(const XML & node, Ts &&... xs)
  // clang-format off
  : Element(
      choice(node,
        std::make_pair(         "WorldPosition", [&](auto && node) { return make<        WorldPosition>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair( "RelativeWorldPosition", [&](auto && node) { return make<RelativeWorldPosition>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair("RelativeObjectPosition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
        std::make_pair(          "RoadPosition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
        std::make_pair(  "RelativeRoadPosition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
        std::make_pair(          "LanePosition", [&](auto && node) { return make<         LanePosition>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair(  "RelativeLanePosition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
        std::make_pair(         "RoutePosition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; })))
  // clang-format on
  {
  }

  explicit operator geometry_msgs::msg::Pose() const;
};

DEFINE_LAZY_VISITOR(
  Position,
  CASE(WorldPosition),          //
  CASE(RelativeWorldPosition),  //
  // CASE(RelativeObjectPosition),
  // CASE(RoadPosition),
  // CASE(RelativeRoadPosition),
  CASE(LanePosition),
  // CASE(RelativeLanePosition),
  // CASE(RoutePosition),
);

DEFINE_LAZY_VISITOR(
  const Position,
  CASE(WorldPosition),          //
  CASE(RelativeWorldPosition),  //
  // CASE(RelativeObjectPosition),
  // CASE(RoadPosition),
  // CASE(RelativeRoadPosition),
  CASE(LanePosition),
  // CASE(RelativeLanePosition),
  // CASE(RoutePosition),
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_
