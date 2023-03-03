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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_DECLARATION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_DECLARATION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/parameter_type.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/value_constraint_group.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ParameterDeclaration ---------------------------------------------------
 *
 *  <xsd:complexType name="ParameterDeclaration">
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="parameterType" type="ParameterType" use="required"/>
 *    <xsd:attribute name="value" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterDeclaration
{
  const String name;

  const ParameterType parameter_type;

  std::list<ValueConstraintGroup> constraint_groups;

  const String value;

  explicit ParameterDeclaration() = default;

  explicit ParameterDeclaration(const pugi::xml_node &, Scope &);

  auto evaluate() const -> Object;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_DECLARATION_HPP_
