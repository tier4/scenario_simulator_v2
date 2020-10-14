// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__TRANSITION_DYNAMICS_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__TRANSITION_DYNAMICS_HPP_

#include <open_scenario_interpreter/syntax/dynamics_dimension.hpp>
#include <open_scenario_interpreter/syntax/dynamics_shape.hpp>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== TransitionDynamics ===================================================
 *
 * <xsd:complexType name="TransitionDynamics">
 *   <xsd:attribute name="dynamicsShape" type="DynamicsShape" use="required"/>
 *   <xsd:attribute name="value" type="Double" use="required"/>
 *   <xsd:attribute name="dynamicsDimension" type="DynamicsDimension" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct TransitionDynamics
{
  const DynamicsShape dynamics_shape;

  const Double value;

  const DynamicsDimension dynamics_dimension;

  template<typename Node, typename Scope>
  explicit TransitionDynamics(const Node & node, Scope & scope)
  : dynamics_shape{readAttribute<DynamicsShape>("dynamicsShape", node, scope)},
    value{readAttribute<Double>("value", node, scope)},
    dynamics_dimension{readAttribute<DynamicsDimension>("dynamicsDimension", node, scope)}
  {}
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__TRANSITION_DYNAMICS_HPP_
