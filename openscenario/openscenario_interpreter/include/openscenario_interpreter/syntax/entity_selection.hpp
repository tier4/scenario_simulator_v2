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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_SELECTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_SELECTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/selected_entities.hpp>
#include <set>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- EntitySelection --------------------------------------------------------
 *
 *  <xsd:complexType name="EntitySelection">
 *    <xsd:sequence>
 *      <xsd:element name="Members" type="SelectedEntities"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct EntitySelection : public Scope, public SelectedEntities
{
  explicit EntitySelection(const pugi::xml_node &, Scope &);

  auto objects() const -> std::set<Entity>;

  auto objectTypes() const -> std::set<ObjectType::value_type>;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_SELECTION_HPP_
