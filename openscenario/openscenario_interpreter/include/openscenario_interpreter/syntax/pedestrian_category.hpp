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
#include <traffic_simulator_msgs/msg/entity_subtype.hpp>

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
 *          <xsd:enumeration value="animal"/>
 *          <xsd:enumeration value="pedestrian"/>
 *          <xsd:enumeration value="wheelchair"/>
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
    pedestrian,  // NOTE: This is the default value and should not be included in the sort.

    // NOTE: Sorted by lexicographic order.
    animal,
    wheelchair,
  } value;

  explicit constexpr PedestrianCategory(value_type value = pedestrian) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }

  explicit operator traffic_simulator_msgs::msg::EntitySubtype() const
  {
    traffic_simulator_msgs::msg::EntitySubtype result;
    {
      switch (value) {
        case pedestrian:
          result.value = traffic_simulator_msgs::msg::EntitySubtype::PEDESTRIAN;
          break;
        default:  // animal, wheelchair
          result.value = traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN;
          break;
      }
    }

    return result;
  }
};

auto operator>>(std::istream &, PedestrianCategory &) -> std::istream &;

auto operator<<(std::ostream &, const PedestrianCategory &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PEDESTRIAN_CATEGORY_HPP_
