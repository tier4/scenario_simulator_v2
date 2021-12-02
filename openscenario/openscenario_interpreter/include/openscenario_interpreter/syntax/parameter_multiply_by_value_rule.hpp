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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_MULTIPLY_BY_VALUE_RULE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_MULTIPLY_BY_VALUE_RULE_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ParameterMultiplyByValueRule -------------------------------------------
 *
 * <xsd:complexType name="ParameterMultiplyByValueRule">
 *   <xsd:attribute name="value" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterMultiplyByValueRule
{
  const Double value;

  explicit ParameterMultiplyByValueRule(const pugi::xml_node &, Scope &);

  auto operator()(const Object &) const -> Object;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_MULTIPLY_BY_VALUE_RULE_HPP_
