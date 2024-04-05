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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_TYPE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_TYPE_HPP_

#include <iostream>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- StoryboardElementType --------------------------------------------------
 *
 *  <xsd:simpleType name="StoryboardElementType">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="story"/>
 *          <xsd:enumeration value="act"/>
 *          <xsd:enumeration value="maneuver"/>
 *          <xsd:enumeration value="event"/>
 *          <xsd:enumeration value="action"/>
 *          <xsd:enumeration value="maneuverGroup"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct StoryboardElementType
{
  enum value_type {
    act,
    action,
    event,
    maneuver,
    maneuverGroup,
    story,
  } value;

  StoryboardElementType() = default;

  constexpr operator value_type() const noexcept { return value; }
};

auto operator>>(std::istream &, StoryboardElementType &) -> std::istream &;

auto operator<<(std::ostream &, const StoryboardElementType &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_TYPE_HPP_
