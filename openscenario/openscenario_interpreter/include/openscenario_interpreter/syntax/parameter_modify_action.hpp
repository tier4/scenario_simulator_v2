// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_MODIFY_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_MODIFY_ACTION_HPP_

#include <openscenario_interpreter/syntax/modify_rule.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ModifyAction -----------------------------------------------------------
 *
 * <xsd:complexType name="ParameterModifyAction">
 *   <xsd:all>
 *     <xsd:element name="Rule" type="ModifyRule"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterModifyAction
{
  Scope inner_scope;

  const String parameter_ref;

  const ModifyRule rule;

  template<typename Node>
  explicit ParameterModifyAction(
    const Node & node, Scope & outer_scope, const String & parameter_ref)
  : inner_scope(outer_scope),
    parameter_ref(parameter_ref),
    rule(readElement<ModifyRule>("Rule", node, inner_scope))
  {}

  auto evaluate() try
  {
    const auto target {
      inner_scope.parameters.at(parameter_ref)
    };
    if (rule.is<ParameterAddValueRule>()) {
      return rule.as<ParameterAddValueRule>()(target);
    } else if (rule.is<ParameterMultiplyByValueRule>()) {
      return rule.as<ParameterMultiplyByValueRule>()(target);
    } else {
      THROW_IMPLEMENTATION_FAULT();
    }
    return unspecified;
  } catch (const std::out_of_range &) {
    throw SemanticError("No such parameter '" + parameter_ref + "'");
  }

  static constexpr auto accomplished() noexcept
  {
    return true;
  }
};
}  // inline namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_MODIFY_ACTION_HPP_
