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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/speed_target_value_type.hpp>

namespace openscenario_interpreter
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

  template <typename Node, typename Scope>
  explicit RelativeTargetSpeed(const Node & node, Scope & scope)
  : entity_ref{readAttribute<String>("entityRef", node, scope)},
    value{readAttribute<Double>("value", node, scope)},
    speed_target_value_type{readAttribute<SpeedTargetValueType>(
      "speedTargetValueType", node, scope, SpeedTargetValueType())},
    continuous{readAttribute<Boolean>("continuous", node, scope, Boolean())}
  {
  }

  std::function<double()> getCalculateAbsoluteTargetSpeed() const
  {
    if (speed_target_value_type == SpeedTargetValueType::factor) {
      return [factor = value, entity_ref = entity_ref]() -> double {
        return factor * getEntityStatus(entity_ref).action_status.twist.linear.x;
      };
    } else if (speed_target_value_type == SpeedTargetValueType::delta) {
      return [delta = value, entity_ref = entity_ref]() -> double {
        return delta + getEntityStatus(entity_ref).action_status.twist.linear.x;
      };
    } else {
      throw UNSUPPORTED_SETTING_DETECTED(RelativeTargetSpeed, speed_target_value_type);
    }
  }

  std::function<bool(const Scope::Actor & actor)> getIsEnd() const
  {
    if (continuous) {
      return [](const auto &) { return false; };  // ends never
    } else {
      return [calc_absolute_target_speed =
                getCalculateAbsoluteTargetSpeed()](const Scope::Actor & actor) {
        try {
          const auto compare = Rule(Rule::equalTo);
          return compare(
            getEntityStatus(actor).action_status.twist.linear.x, calc_absolute_target_speed());
        } catch (const SemanticError &) {
          return false;  // NOTE: The actor is maybe lane-changing now
        }
      };
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_TARGET_SPEED_HPP_
