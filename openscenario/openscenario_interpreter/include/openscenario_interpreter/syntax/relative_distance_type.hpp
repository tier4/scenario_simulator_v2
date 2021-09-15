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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_TYPE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_TYPE_HPP_

#include <iostream>
#include <type_traits>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- RelativeDistanceType (OpenSCENARIO 1.1) --------------------------------
 *
 *  <xsd:simpleType name="RelativeDistanceType">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="lateral"/>
 *          <xsd:enumeration value="longitudinal"/>
 *          <xsd:enumeration value="cartesianDistance">
 *            <xsd:annotation>
 *              <xsd:appinfo>
 *                deprecated
 *              </xsd:appinfo>
 *            </xsd:annotation>
 *          </xsd:enumeration>
 *          <xsd:enumeration value="euclidianDistance"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct RelativeDistanceType
{
  enum value_type {
    longitudinal,
    lateral,
    euclidianDistance,
  } value;

  explicit RelativeDistanceType() = default;

  constexpr operator value_type() const noexcept { return value; }
};

static_assert(std::is_standard_layout<RelativeDistanceType>::value, "");

static_assert(std::is_trivial<RelativeDistanceType>::value, "");

std::istream & operator>>(std::istream &, RelativeDistanceType &);

std::ostream & operator<<(std::ostream &, const RelativeDistanceType &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_TYPE_HPP_
