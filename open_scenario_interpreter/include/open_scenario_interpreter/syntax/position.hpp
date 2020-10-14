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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_

#include <open_scenario_interpreter/syntax/lane_position.hpp>
#include <open_scenario_interpreter/syntax/world_position.hpp>

#include <utility>

namespace open_scenario_interpreter
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
  : public Element
{
  template<typename Node, typename ... Ts>
  explicit Position(const Node & node, Ts && ... xs)
  : Element(
      choice(
        node,
        std::make_pair("WorldPosition", [&](auto && node) {
          return make<WorldPosition>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("RelativeWorldPosition", UNSUPPORTED()),
        std::make_pair("RelativeObjectPosition", UNSUPPORTED()),
        std::make_pair("RoadPosition", UNSUPPORTED()),
        std::make_pair("RelativeRoadPosition", UNSUPPORTED()),
        std::make_pair("LanePosition", [&](auto && node) {
          return make<LanePosition>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("RelativeLanePosition", UNSUPPORTED()),
        std::make_pair("RoutePosition", UNSUPPORTED())))
  {}
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_
