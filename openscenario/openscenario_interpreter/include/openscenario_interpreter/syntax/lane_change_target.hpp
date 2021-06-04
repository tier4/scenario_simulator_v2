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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__LANE_CHANGE_TARGET_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__LANE_CHANGE_TARGET_HPP_

#include <openscenario_interpreter/syntax/absolute_target_lane.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- LaneChangeTarget -------------------------------------------------------
 *
 *  <xsd:complexType name="LaneChangeTarget">
 *    <xsd:choice>
 *      <xsd:element name="RelativeTargetLane" type="RelativeTargetLane"/>
 *      <xsd:element name="AbsoluteTargetLane" type="AbsoluteTargetLane"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct LaneChangeTarget : public ComplexType
{
  template <typename Node, typename... Ts>
  explicit LaneChangeTarget(const Node & node, Ts &&... xs)
  // clang-format off
  : ComplexType(
      choice(node,
        std::make_pair("RelativeTargetLane", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
        std::make_pair("AbsoluteTargetLane", [&](auto && node) { return make<AbsoluteTargetLane>(node, std::forward<decltype(xs)>(xs)...); })))
  // clang-format on
  {
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__LANE_CHANGE_TARGET_HPP_
