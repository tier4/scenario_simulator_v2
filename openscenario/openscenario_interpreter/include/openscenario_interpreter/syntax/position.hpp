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
#define ELEMENT(TYPE) \
  std::make_pair(     \
    #TYPE, [&](auto && node) { return make<TYPE>(node, std::forward<decltype(xs)>(xs)...); })

struct Position : public Element
{
  template <typename XML, typename... Ts>
  explicit Position(const XML & node, Ts &&... xs)
  // clang-format off
  : Element(
      choice(node,
        std::make_pair(         "WorldPosition", [&](auto && node) { return make<        WorldPosition>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair( "RelativeWorldPosition", [&](auto && node) { return make<RelativeWorldPosition>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair("RelativeObjectPosition", UNSUPPORTED()),
        std::make_pair(          "RoadPosition", UNSUPPORTED()),
        std::make_pair(  "RelativeRoadPosition", UNSUPPORTED()),
        std::make_pair(          "LanePosition", [&](auto && node) { return make<         LanePosition>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair(  "RelativeLanePosition", UNSUPPORTED()),
        std::make_pair(         "RoutePosition", UNSUPPORTED())))
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
      throw UNSUPPORTED_ELEMENT_SPECIFIED(Position, type().name());
    }
  }
};

#undef ELEMENT

template <typename R = void, typename F, typename... Ts>
decltype(auto) apply(F && f, const Position & position, Ts &&... xs)
{
#define BOILERPLATE(TYPE)                                               \
  {                                                                     \
    typeid(TYPE), [](F && f, const Position & position, Ts &&... xs) {  \
      return f(position.as<TYPE>(), std::forward<decltype(xs)>(xs)...); \
    }                                                                   \
  }

  // clang-format off
  static const std::unordered_map<
    std::type_index, std::function<R(F && f, const Position & position, Ts &&... xs)>>
  overloads
  {
    BOILERPLATE(         WorldPosition),
    BOILERPLATE( RelativeWorldPosition),
    // BOILERPLATE(RelativeObjectPosition),
    // BOILERPLATE(          RoadPosition),
    // BOILERPLATE(  RelativeRoadPosition),
    BOILERPLATE(          LanePosition),
    // BOILERPLATE(  RelativeLanePosition),
    // BOILERPLATE(         RoutePosition),
  };
  // clang-format on

#undef BOILERPLATE

  return overloads.at(position.type())(
    std::forward<decltype(f)>(f), position, std::forward<decltype(xs)>(xs)...);
}
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__POSITION_HPP_
