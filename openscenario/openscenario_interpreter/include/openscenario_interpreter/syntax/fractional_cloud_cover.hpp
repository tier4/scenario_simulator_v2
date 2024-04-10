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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__FRACTIONAL_CLOUD_COVER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__FRACTIONAL_CLOUD_COVER_HPP_

#include <iostream>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- FractionalCloudCover 1.2 ----------------------------------------
 *
 * <xsd:simpleType name="FractionalCloudCover">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="zeroOktas"/>
 *         <xsd:enumeration value="oneOktas"/>
 *         <xsd:enumeration value="twoOktas"/>
 *         <xsd:enumeration value="threeOktas"/>
 *         <xsd:enumeration value="fourOktas"/>
 *         <xsd:enumeration value="fiveOktas"/>
 *         <xsd:enumeration value="sixOktas"/>
 *         <xsd:enumeration value="sevenOktas"/>
 *         <xsd:enumeration value="eightOktas"/>
 *         <xsd:enumeration value="nineOktas"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct FractionalCloudCover
{
  enum value_type {
    zeroOktas,
    oneOktas,
    twoOktas,
    threeOktas,
    fourOktas,
    fiveOktas,
    sixOktas,
    sevenOktas,
    eightOktas,
    nineOktas
  } value;

  FractionalCloudCover() = default;

  constexpr FractionalCloudCover(value_type value) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }
};

auto operator>>(std::istream &, FractionalCloudCover &) -> std::istream &;

auto operator<<(std::ostream &, const FractionalCloudCover &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__FRACTIONAL_CLOUD_COVER_HPP_
