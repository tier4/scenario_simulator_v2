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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ADD_ENTITY_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ADD_ENTITY_ACTION_HPP_

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/syntax/string.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- AddEntityAction --------------------------------------------------------
 *
 *  <xsd:complexType name="AddEntityAction">
 *    <xsd:all>
 *      <xsd:element name="Position" type="Position"/>
 *    </xsd:all>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct AddEntityAction
{
  const Position position;

  template <typename Node, typename Scope>
  explicit AddEntityAction(const Node & node, Scope & outer_scope)
  : position(readElement<Position>("Position", node, outer_scope))
  {
  }

  decltype(auto) operator()(const String & entity_ref) const
  {
    std::cout << "AddEntityAction: " << entity_ref << std::endl;
    return unspecified;
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ADD_ENTITY_ACTION_HPP_
