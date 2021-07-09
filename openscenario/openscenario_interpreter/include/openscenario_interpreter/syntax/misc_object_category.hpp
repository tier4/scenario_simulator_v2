// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__MISC_OBJECT_CATEGORY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__MISC_OBJECT_CATEGORY_HPP_

#include <iostream>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- MiscObjectCategory 1.1 -------------------------------------------------
 *
 * <xsd:simpleType name="MiscObjectCategory">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="barrier"/>
 *         <xsd:enumeration value="building"/>
 *         <xsd:enumeration value="crosswalk"/>
 *         <xsd:enumeration value="gantry"/>
 *         <xsd:enumeration value="none"/>
 *         <xsd:enumeration value="obstacle"/>
 *         <xsd:enumeration value="parkingSpace"/>
 *         <xsd:enumeration value="patch"/>
 *         <xsd:enumeration value="pole"/>
 *         <xsd:enumeration value="railing"/>
 *         <xsd:enumeration value="roadMark"/>
 *         <xsd:enumeration value="soundBarrier"/>
 *         <xsd:enumeration value="streetLamp"/>
 *         <xsd:enumeration value="trafficIsland"/>
 *         <xsd:enumeration value="tree"/>
 *         <xsd:enumeration value="vegetation"/>
 *         <xsd:enumeration value="wind">
 *           <xsd:annotation>
 *             <xsd:appinfo>
 *               deprecated
 *             </xsd:appinfo>
 *           </xsd:annotation>
 *         </xsd:enumeration>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 * -------------------------------------------------------------------------- */
struct MiscObjectCategory
{
  enum value_type {
    barrier,
    building,
    crosswalk,
    gantry,
    none,
    obstacle,
    parkingSpace,
    patch,
    pole,
    railing,
    roadMark,
    soundBarrier,
    streetLamp,
    trafficIsland,
    tree,
    vegetation,
    wind,  // NOTE: DEPRECATED (since OpenSCENARIO 1.1)
  } value;

  explicit MiscObjectCategory() = default;

  constexpr operator value_type() const noexcept { return value; }
};

static_assert(std::is_standard_layout<MiscObjectCategory>::value, "");

static_assert(std::is_trivial<MiscObjectCategory>::value, "");

auto operator>>(std::istream &, MiscObjectCategory &) -> std::istream &;

auto operator<<(std::ostream &, const MiscObjectCategory &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__MISC_OBJECT_CATEGORY_HPP_
