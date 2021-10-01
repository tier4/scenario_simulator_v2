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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_CONDITION_HPP_

#include <iomanip>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <typeindex>
#include <unordered_map>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ParameterCondition -----------------------------------------------------
 *
 *  <xsd:complexType name="ParameterCondition">
 *    <xsd:attribute name="parameterRef" type="String" use="required"/>
 *    <xsd:attribute name="value" type="String" use="required"/>
 *    <xsd:attribute name="rule" type="Rule" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterCondition : private Scope
{
  const String parameter_ref;

  const String value;

  const Rule compare;

  template <typename Node>
  explicit ParameterCondition(const Node & node, Scope & current_scope)
  // clang-format off
  : Scope(current_scope),
    parameter_ref(readAttribute<String>("parameterRef", node, localScope())),
    value        (readAttribute<String>("value",        node, localScope())),
    compare      (readAttribute<Rule>  ("rule",         node, localScope()))
  // clang-format on
  {
  }

  auto description() const -> String;

  auto evaluate() const -> Element;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_CONDITION_HPP_
