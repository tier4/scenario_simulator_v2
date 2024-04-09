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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DYNAMICS_DIMENSION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DYNAMICS_DIMENSION_HPP_

#include <iostream>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/data_type/speed_change.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- DynamicsDimension ------------------------------------------------------
 *
 *  <xsd:simpleType name="DynamicsDimension">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="rate"/>
 *          <xsd:enumeration value="time"/>
 *          <xsd:enumeration value="distance"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct DynamicsDimension
{
  enum value_type {
    // A predefined constant rate is used to acquire the target value.
    rate,

    // A predefined time (duration) is used to acquire the target value.
    time,

    // A predefined distance used to acquire the target value.
    distance,
  } value;

  DynamicsDimension() = default;

  constexpr operator value_type() const noexcept { return value; }

  explicit constexpr operator traffic_simulator::speed_change::Constraint::Type() const
  {
    switch (value) {
      case rate:
        return traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION;
      default:
        return {};
    }
  }

  explicit constexpr operator traffic_simulator::lane_change::Constraint::Type() const
  {
    switch (value) {
      case rate:
        return traffic_simulator::lane_change::Constraint::Type::LATERAL_VELOCITY;
      case time:
        return traffic_simulator::lane_change::Constraint::Type::TIME;
      case distance:
        return traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE;
      default:
        return {};
    }
  }
};

auto operator>>(std::istream &, DynamicsDimension &) -> std::istream &;

auto operator<<(std::ostream &, const DynamicsDimension &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DYNAMICS_DIMENSION_HPP_
