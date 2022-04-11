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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_CATEGORY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_CATEGORY_HPP_

#include <iostream>
#include <traffic_simulator_msgs/msg/entity_subtype.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- VehicleCategory --------------------------------------------------------
 *
 *  <xsd:simpleType name="VehicleCategory">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="bicycle"/>
 *          <xsd:enumeration value="bus"/>
 *          <xsd:enumeration value="car"/>
 *          <xsd:enumeration value="motorbike"/>
 *          <xsd:enumeration value="semitrailer"/>
 *          <xsd:enumeration value="trailer"/>
 *          <xsd:enumeration value="train"/>
 *          <xsd:enumeration value="tram"/>
 *          <xsd:enumeration value="truck"/>
 *          <xsd:enumeration value="van"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct VehicleCategory
{
  enum value_type {
    car,  // NOTE: This is the default value and should not be included in the sort.

    // NOTE: Sorted by lexicographic order.
    bicycle,
    bus,
    motorbike,
    semitrailer,
    trailer,
    train,
    tram,
    truck,
    van,
  } value;

  explicit constexpr VehicleCategory(value_type value = car) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }

  explicit operator traffic_simulator_msgs::msg::EntitySubtype() const
  {
    traffic_simulator_msgs::msg::EntitySubtype result;
    {
      switch (value) {  // NOTE: Sorted by lexicographic order.
        case bicycle:
          result.value = traffic_simulator_msgs::msg::EntitySubtype::BICYCLE;
          break;
        case bus:
          result.value = traffic_simulator_msgs::msg::EntitySubtype::BUS;
          break;
        case car:
          result.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
          break;
        case motorbike:
          result.value = traffic_simulator_msgs::msg::EntitySubtype::MOTORCYCLE;
          break;
        case trailer:
          result.value = traffic_simulator_msgs::msg::EntitySubtype::TRAILER;
          break;
        case truck:
          result.value = traffic_simulator_msgs::msg::EntitySubtype::TRUCK;
          break;
        default:  // semitrailer, train, tram, van
          result.value = traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN;
          break;
      }
    }

    return result;
  }
};

auto operator>>(std::istream &, VehicleCategory &) -> std::istream &;

auto operator<<(std::ostream &, const VehicleCategory &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__VEHICLE_CATEGORY_HPP_
