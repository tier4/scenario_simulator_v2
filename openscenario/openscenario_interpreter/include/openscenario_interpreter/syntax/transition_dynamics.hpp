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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRANSITION_DYNAMICS_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRANSITION_DYNAMICS_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/dynamics_dimension.hpp>
#include <openscenario_interpreter/syntax/dynamics_shape.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- TransitionDynamics -----------------------------------------------------
 *
 * <xsd:complexType name="TransitionDynamics">
 *   <xsd:attribute name="dynamicsShape" type="DynamicsShape" use="required"/>
 *   <xsd:attribute name="value" type="Double" use="required"/>
 *   <xsd:attribute name="dynamicsDimension" type="DynamicsDimension" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct TransitionDynamics
{
  const DynamicsShape dynamics_shape;

  const Double value;

  const DynamicsDimension dynamics_dimension;

  explicit TransitionDynamics(const pugi::xml_node &, Scope &);

  explicit operator traffic_simulator::speed_change::Constraint() const
  {
    return traffic_simulator::speed_change::Constraint(
      static_cast<traffic_simulator::speed_change::Constraint::Type>(dynamics_dimension),
      static_cast<double>(value));
  }

  explicit operator traffic_simulator::lane_change::Constraint() const
  {
    return traffic_simulator::lane_change::Constraint(
      static_cast<traffic_simulator::lane_change::Constraint::Type>(dynamics_dimension),
      static_cast<double>(value));
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRANSITION_DYNAMICS_HPP_
