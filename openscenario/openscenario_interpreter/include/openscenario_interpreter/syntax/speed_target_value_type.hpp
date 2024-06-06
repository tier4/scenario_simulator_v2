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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_TARGET_VALUE_TYPE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_TARGET_VALUE_TYPE_HPP_

#include <iostream>
#include <traffic_simulator/data_type/speed_change.hpp>
#include <type_traits>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- SpeedTargetValueType ---------------------------------------------------
 *
 *  <xsd:simpleType name="SpeedTargetValueType">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="delta"/>
 *          <xsd:enumeration value="factor"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct SpeedTargetValueType
{
  enum value_type {
    /*
       The relative value is interpreted as a difference to a referenced value.
       Unit: m/s. As an example, a speed value of 10 equals a speed that's 10m/s
       faster than the reference speed.
    */
    delta,

    /*
       The relative value is interpreted as a factor to a referenced value. No
       unit. As an example, a speed value of 1.1 equals a speed that's 10%
       faster than the reference speed.
    */
    factor,
  } value;

  SpeedTargetValueType() = default;

  constexpr operator value_type() const noexcept { return value; }

  explicit constexpr operator traffic_simulator::speed_change::RelativeTargetSpeed::Type() const
  {
    switch (value) {
      case delta:
        return traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA;
      case factor:
        return traffic_simulator::speed_change::RelativeTargetSpeed::Type::FACTOR;
      default:
        return traffic_simulator::speed_change::RelativeTargetSpeed::Type();
    }
  }
};

auto operator>>(std::istream &, SpeedTargetValueType &) -> std::istream &;

auto operator<<(std::ostream &, const SpeedTargetValueType &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_TARGET_VALUE_TYPE_HPP_
