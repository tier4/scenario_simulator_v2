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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__COORDINATE_SYSTEM_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__COORDINATE_SYSTEM_HPP_

#include <iostream>
#include <type_traits>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- CoordinateSystem (OpenSCENARIO 1.1) ------------------------------------
 *
 *  <xsd:simpleType name="CoordinateSystem">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="entity"/>
 *          <xsd:enumeration value="lane"/>
 *          <xsd:enumeration value="road"/>
 *          <xsd:enumeration value="trajectory"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct CoordinateSystem
{
  enum value_type {
    entity,
    lane,
    road,
    trajectory,
  } value;

  CoordinateSystem() = default;

  constexpr CoordinateSystem(value_type value) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }
};

auto operator>>(std::istream &, CoordinateSystem &) -> std::istream &;

auto operator<<(std::ostream &, const CoordinateSystem &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__COORDINATE_SYSTEM_HPP_
