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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SELECTED_ENTITIES_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SELECTED_ENTITIES_HPP_

#include <openscenario_interpreter/object.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/by_type.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- SelectedEntities -------------------------------------------------------
 *
 *  <xsd:complexType name="SelectedEntities">
 *    <xsd:choice>
 *      <xsd:element name="EntityRef" minOccurs="0" maxOccurs="unbounded" type="EntityRef"/>
 *      <xsd:element name="ByType" minOccurs="0" maxOccurs="unbounded" type="ByType"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct SelectedEntities : public ComplexType
{
  const std::list<Entity> entityRef;

  const std::list<ByType> byType;

  explicit SelectedEntities(const pugi::xml_node &, Scope &);
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SELECTED_ENTITIES_HPP_
