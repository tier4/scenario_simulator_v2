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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_

#include <open_scenario_interpreter/syntax/speed_target_value_type.hpp>
#include <open_scenario_interpreter/reader/attribute.hpp>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== RelativeTargetSpeed ==================================================
 *
 * <xsd:complexType name="RelativeTargetSpeed">
 *   <xsd:attribute name="entityRef" type="String" use="required"/>
 *   <xsd:attribute name="value" type="Double" use="required"/>
 *   <xsd:attribute name="speedTargetValueType" type="SpeedTargetValueType" use="required"/>
 *   <xsd:attribute name="continuous" type="Boolean" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct RelativeTargetSpeed
{
  const String entity_ref;

  const Double value;

  const SpeedTargetValueType speed_target_value_type;

  const Boolean continuous;

  template<typename Node, typename Scope>
  explicit RelativeTargetSpeed(const Node & node, Scope & scope)
  : entity_ref{readAttribute<String>("entityRef", node, scope)},
    value{readAttribute<Double>("value", node, scope)},
    speed_target_value_type{
      readAttribute<SpeedTargetValueType>(
        "speedTargetValueType", node, scope, SpeedTargetValueType())},
    continuous{readAttribute<Boolean>("continuous", node, scope, Boolean())}
  {}
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_
