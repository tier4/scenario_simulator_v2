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
#define ELEMENT(TYPE) \
  std::make_pair( \
    #TYPE, [&](auto && node) \
    { \
      return make<TYPE>(node, std::forward<decltype(xs)>(xs)...); \
    })

struct Position
  : public Element
{
  template<typename Node, typename ... Ts>
  explicit Position(const Node & node, Ts && ... xs)
  : Element(
      choice(
        node,
        ELEMENT(/*   */ WorldPosition),
        ELEMENT(RelativeWorldPosition),
        std::make_pair("RelativeObjectPosition", UNSUPPORTED()),
        std::make_pair("RoadPosition", UNSUPPORTED()),
        std::make_pair("RelativeRoadPosition", UNSUPPORTED()),
        ELEMENT(LanePosition),
        std::make_pair("RelativeLanePosition", UNSUPPORTED()),
        std::make_pair("RoutePosition", UNSUPPORTED())))
  {}

  geometry_msgs::msg::Pose toPose() const
  {
    if ((*this).is<WorldPosition>()) {
      return (*this).as<WorldPosition>();
    } else if ((*this).is<LanePosition>()) {
      return (*this).as<LanePosition>();
    } else {
      const geometry_msgs::msg::Pose result {};
      return result;
    }
  }
};

#undef ELEMENT
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_
