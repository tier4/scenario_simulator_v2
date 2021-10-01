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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_DECLARATION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_DECLARATION_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <string>
#include <vector>

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

  const String value;

  ParameterDeclaration() = default;

  template <typename Node>
  explicit ParameterDeclaration(const Node & node, Scope & scope)
  // clang-format off
  : name          (readAttribute<String       >("name",          node, scope)),
    parameter_type(readAttribute<ParameterType>("parameterType", node, scope)),
    value         (readAttribute<String       >("value",         node, scope))
  // clang-format on
  {
    if (name.substr(0, 3) == "OSC") {
      throw SyntaxError(
        "Parameter names starting with \"OSC\" are reserved for special use in future versions "
        "of OpenSCENARIO. Generally, it is forbidden to use the OSC prefix.");
    } else if (includes(name, {' ', '$', '\'', '"'})) {
      throw SyntaxError(
        "In parameter names, usage of symbols is restricted. Symbols that must not be used are:\n"
        "  - \" \" (blank space)\n"
        "  - $\n"
        "  - \'\n"
        "  - \"\n");
    } else {
      scope.insert(name, evaluate());
    }
  }

  auto evaluate() const -> Element;

  auto includes(const std::string &, const std::vector<char> &) -> bool;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_DECLARATION_HPP_
