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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__OBJECT_TYPE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__OBJECT_TYPE_HPP_

#include <istream>
#include <ostream>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ObjectType -------------------------------------------------------------
 *
 *  <xsd:simpleType name="ObjectType">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="pedestrian"/>
 *          <xsd:enumeration value="vehicle"/>
 *          <xsd:enumeration value="miscellaneous"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct ObjectType
{
  enum value_type {
    vehicle,

    external,
    miscellaneous,
    pedestrian,
  } value;

  ObjectType() = default;

  operator value_type() const noexcept { return value; }
};

auto operator>>(std::istream &, ObjectType &) -> std::istream &;

auto operator<<(std::ostream &, const ObjectType &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__OBJECT_TYPE_HPP_
