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

#ifndef SCENARIO_RUNNER__SYNTAX__PARAMETER_MODIFY_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__PARAMETER_MODIFY_ACTION_HPP_

#include <scenario_runner/syntax/modify_rule.hpp>

namespace scenario_runner
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

  const ModifyRule modify;

  template<typename Node>
  explicit ParameterModifyAction(
    const Node & node, Scope & outer_scope, const String & parameter_ref)
  : inner_scope(outer_scope),
    parameter_ref(parameter_ref),
    modify(readElement<ModifyRule>("Rule", node, inner_scope))
  {}

  auto evaluate()
  {
    std::cout << "parameterRef: " << parameter_ref << std::endl;
    std::cout << "value: " << inner_scope.parameters.at(parameter_ref) << std::endl;
    return unspecified;
  }

  static constexpr auto accomplished() noexcept
  {
    return true;
  }
};
}  // inline namespace syntax
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__PARAMETER_MODIFY_ACTION_HPP_
