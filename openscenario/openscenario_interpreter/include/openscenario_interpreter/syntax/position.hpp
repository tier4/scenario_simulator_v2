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

#include <boost/function_types/result_type.hpp>
#include <openscenario_interpreter/syntax/lane_position.hpp>
#include <openscenario_interpreter/syntax/relative_world_position.hpp>
#include <openscenario_interpreter/syntax/world_position.hpp>
#include <stdexcept>
#include <unordered_map>
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

  explicit operator geometry_msgs::msg::Pose() const
  {
    if (is<WorldPosition>()) {
      return static_cast<geometry_msgs::msg::Pose>(as<WorldPosition>());
    } else if (is<LanePosition>()) {
      return static_cast<geometry_msgs::msg::Pose>(as<LanePosition>());
    } else {
      // NOTE: Specifying an unsupported element is an error in the constructor, so this line cannot be reached.
      throw UNSUPPORTED_ELEMENT_SPECIFIED(type().name());
    }
  }
};

template <typename Result = void, typename Function, typename... Ts>
auto apply(Function && function, const Position & position, Ts &&... xs) -> Result
{
  using application = std::function<Result(Function &&, const Position &, Ts &&...)>;

#define BOILERPLATE(TYPE)                                                            \
  std::make_pair<std::type_index, application>(                                      \
    typeid(TYPE), [](Function && function, const Position & position, Ts &&... xs) { \
      return function(position.as<TYPE>(), std::forward<decltype(xs)>(xs)...);       \
    })

  static const std::unordered_map<std::type_index, application> overloads{
    // clang-format off
    BOILERPLATE(         WorldPosition),
    BOILERPLATE( RelativeWorldPosition),
    // BOILERPLATE(RelativeObjectPosition),
    // BOILERPLATE(          RoadPosition),
    // BOILERPLATE(  RelativeRoadPosition),
    BOILERPLATE(          LanePosition),
    // BOILERPLATE(  RelativeLanePosition),
    // BOILERPLATE(         RoutePosition),
    // clang-format on
  };

#undef BOILERPLATE

  try {
    return overloads.at(position.type())(
      std::forward<decltype(function)>(function), position, std::forward<decltype(xs)>(xs)...);
  } catch (const std::out_of_range &) {
    throw UNSUPPORTED_SETTING_DETECTED(Position, makeTypename(position.type().name()));
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_
