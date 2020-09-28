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

#ifndef SCENARIO_RUNNER__SYNTAX__TRANSITION_DYNAMICS_HPP_
#define SCENARIO_RUNNER__SYNTAX__TRANSITION_DYNAMICS_HPP_

#include <scenario_runner/syntax/dynamics_dimension.hpp>
#include <scenario_runner/syntax/dynamics_shape.hpp>

namespace scenario_runner
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
  : dynamics_shape{readAttribute<DynamicsShape>(node, scope, "dynamicsShape")},
    value{readAttribute<Double>(node, scope, "value")},
    dynamics_dimension{readAttribute<DynamicsDimension>(node, scope, "dynamicsDimension")}
  {}
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__TRANSITION_DYNAMICS_HPP_
