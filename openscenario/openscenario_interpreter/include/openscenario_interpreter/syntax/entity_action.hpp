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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/add_entity_action.hpp>
#include <openscenario_interpreter/syntax/delete_entity_action.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- EntityAction -----------------------------------------------------------
 *
 *  <xsd:complexType name="EntityAction">
 *    <xsd:choice>
 *      <xsd:element name="AddEntityAction" type="AddEntityAction"/>
 *      <xsd:element name="DeleteEntityAction" type="DeleteEntityAction"/>
 *    </xsd:choice>
 *    <xsd:attribute name="entityRef" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct EntityAction : public ComplexType
{
  const Entity entity_ref;

  explicit EntityAction(const pugi::xml_node &, Scope &);

  static auto accomplished() noexcept -> bool;

  static auto endsImmediately() noexcept -> bool;

  static auto run() noexcept -> void;

  /*  */ auto start() const -> void;
};

DEFINE_LAZY_VISITOR(
  EntityAction,
  CASE(AddEntityAction),     //
  CASE(DeleteEntityAction),  //
);

DEFINE_LAZY_VISITOR(
  const EntityAction,
  CASE(AddEntityAction),     //
  CASE(DeleteEntityAction),  //
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_ACTION_HPP_
