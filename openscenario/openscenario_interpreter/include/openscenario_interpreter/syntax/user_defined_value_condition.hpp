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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__USER_DEFINED_VALUE_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__USER_DEFINED_VALUE_CONDITION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- UserDefinedValueCondition ----------------------------------------------
 *
 *  This condition acts as a wrapper for external custom conditions which are
 *  implemented in the user software. This condition is considered true if the
 *  given value verifies the specified relation rule (bigger than, smaller than,
 *  or equal to) relatively to the provided reference.
 *
 *  <xsd:complexType name="UserDefinedValueCondition">
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="value" type="String" use="required"/>
 *    <xsd:attribute name="rule" type="Rule" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
class UserDefinedValueCondition : private SimulatorCore::NonStandardOperation
{
  Object result;

  std::function<Object()> evaluate_value;

public:
  const String name;

  const String value;

  const Rule rule;

  explicit UserDefinedValueCondition(const pugi::xml_node &, Scope &);

  auto description() const -> String;

  auto evaluate() -> Object;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__USER_DEFINED_VALUE_CONDITION_HPP_
