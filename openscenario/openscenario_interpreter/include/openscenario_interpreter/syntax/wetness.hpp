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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__WETNESS_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__WETNESS_HPP_

#include <iostream>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Wetness 1.2 --------------------------------------------------------
 *
 * <xsd:simpleType name="Wetness">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="dry"/>
 *         <xsd:enumeration value="moist"/>
 *         <xsd:enumeration value="wetWithPuddles"/>
 *         <xsd:enumeration value="lowFlooded"/>
 *         <xsd:enumeration value="highFlooded"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct Wetness
{
  enum value_type {
    dry,             // Not wet.
    moist,           // Wet but no puddles are formed.
    wetWithPuddles,  // Wet, puddles are formed.
    lowFlooded,      // Road completely covered with water. No puddles anymore.
    highFlooded      // Road completely covered with water. Water depth > 5cm.
  } value;

  Wetness() = default;

  constexpr Wetness(value_type value) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }
};

auto operator>>(std::istream &, Wetness &) -> std::istream &;

auto operator<<(std::ostream &, const Wetness &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__WETNESS_HPP_
