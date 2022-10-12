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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__FOLLOWING_MODE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__FOLLOWING_MODE_HPP_

#include <iostream>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- FollowingMode 1.2 ------------------------------------------------------
 *
 *  <xsd:simpleType name="FollowingMode">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="follow"/>
 *          <xsd:enumeration value="position"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct FollowingMode
{
  enum value_type {
    follow, /*
      Follow the lateral and/or longitudinal target value as good as possible
      by observing the dynamic constraints of the entity (e.g. for a driver
      model). */

    position, /*
      Follow the trajectory, shape or profile exactly by ignoring the dynamic
      constraints of the entity. */
  } value;

  constexpr operator value_type() const noexcept { return value; }
};

auto operator>>(std::istream &, FollowingMode &) -> std::istream &;

auto operator<<(std::ostream &, const FollowingMode &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__FOLLOWING_MODE_HPP_
