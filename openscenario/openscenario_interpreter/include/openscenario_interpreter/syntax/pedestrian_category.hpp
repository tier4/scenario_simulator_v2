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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_CATEGORY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_CATEGORY_HPP_

#include <iostream>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- PedestrianCategory -----------------------------------------------------
 *
 *  <xsd:simpleType name="PedestrianCategory">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="pedestrian"/>
 *          <xsd:enumeration value="wheelchair"/>
 *          <xsd:enumeration value="animal"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct PedestrianCategory
{
  enum value_type {
    pedestrian,
    wheelchair,
    animal,
  } value;

  explicit constexpr PedestrianCategory(value_type value = pedestrian) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }
};

std::istream & operator>>(std::istream &, PedestrianCategory &);

std::ostream & operator<<(std::ostream &, const PedestrianCategory &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_CATEGORY_HPP_
