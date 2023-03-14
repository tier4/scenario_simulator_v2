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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DYNAMIC_CONSTRAINTS_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DYNAMIC_CONSTRAINTS_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <pugixml.hpp>

#include <traffic_simulator_msgs/msg/dynamic_constraints.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- DynamicConstraints 1.2 -------------------------------------------------
 *
 *  <xsd:complexType name="DynamicConstraints">
 *    <xsd:attribute name="maxAcceleration" type="Double"/>
 *    <xsd:attribute name="maxAccelerationRate" type="Double"/>
 *    <xsd:attribute name="maxDeceleration" type="Double"/>
 *    <xsd:attribute name="maxDecelerationRate" type="Double"/>
 *    <xsd:attribute name="maxSpeed" type="Double"/>
 *  </xsd:complexType>
 *
 *  syntax `Vehicle.Performance` has exactly the same elements as
 *  this syntax `DynamicConstraints`, but `Vehicle.Performance` expresses the
 *  maximum design performance of the vehicle, whereas `DynamicConstraints`
 *  expresses how much of it the driver is allowed to use.
 *
 * -------------------------------------------------------------------------- */
struct DynamicConstraints
{
  const Double max_acceleration;

  const Double max_acceleration_rate;

  const Double max_deceleration;

  const Double max_deceleration_rate;

  const Double max_speed;

  const Double max_jerk;

  explicit DynamicConstraints(
    const Double max_acceleration = Double::infinity(),
    const Double max_acceleration_rate = Double::infinity(),
    const Double max_deceleration = Double::infinity(),
    const Double max_deceleration_rate = Double::infinity(),
    const Double max_speed = Double::infinity(), const Double max_jerk = Double::infinity());

  explicit DynamicConstraints(const pugi::xml_node &, Scope &);

  explicit operator traffic_simulator_msgs::msg::DynamicConstraints() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DYNAMIC_CONSTRAINTS_HPP_
