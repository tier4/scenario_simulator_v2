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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PROPERTY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PROPERTY_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/string.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Property ---------------------------------------------------------------
 *
 *  <xsd:complexType name="Property">
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="value" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Property
{
  using Name = String;

  const Name name;

  using Value = String;

  const Value value;

  template
  <
    typename Node, typename Scope
  >
  explicit Property(const Node & node, Scope & outer_scope)
  : name(readAttribute<Name>("name", node, outer_scope)),
    value(readAttribute<Value>("value", node, outer_scope))
  {}
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PROPERTY_HPP_
