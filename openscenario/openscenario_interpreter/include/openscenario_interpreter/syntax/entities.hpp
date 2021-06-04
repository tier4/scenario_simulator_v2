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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ENTITIES_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ENTITIES_HPP_

#include <openscenario_interpreter/syntax/entity_selection.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Entities ---------------------------------------------------------------
 *
 *  <xsd:complexType name="Entities">
 *    <xsd:sequence>
 *      <xsd:element name="ScenarioObject" minOccurs="0" maxOccurs="unbounded" type="ScenarioObject"/>
 *      <xsd:element name="EntitySelection" minOccurs="0" maxOccurs="unbounded" type="EntitySelection"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Entities
{
  template <typename Node, typename Scope>
  explicit Entities(const Node & node, Scope & outer_scope)
  {
    callWithElements(node, "ScenarioObject", 0, unbounded, [&](auto && node) {
      outer_scope.entities.emplace(
        readAttribute<ScenarioObject::Name>("name", node, outer_scope),
        make<ScenarioObject>(node, outer_scope));
    });

    callWithElements(node, "EntitySelection", 0, unbounded, [&](auto && node) {
      throw UNSUPPORTED_ELEMENT_SPECIFIED(Entities, node.name());
      return unspecified;
    });
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITIES_HPP_
