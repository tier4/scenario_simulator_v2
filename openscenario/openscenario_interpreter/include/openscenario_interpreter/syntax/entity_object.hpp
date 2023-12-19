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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_OBJECT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_OBJECT_HPP_

#include <openscenario_interpreter/syntax/misc_object.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/pedestrian.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- EntityObject -----------------------------------------------------------
 *
 *  <xsd:group name="EntityObject">
 *    <xsd:choice>
 *      <xsd:element name="CatalogReference" type="CatalogReference"/>
 *      <xsd:element name="Vehicle" type="Vehicle"/>
 *      <xsd:element name="Pedestrian" type="Pedestrian"/>
 *      <xsd:element name="MiscObject" type="MiscObject"/>
 *    </xsd:choice>
 *  </xsd:group>
 *
 * -------------------------------------------------------------------------- */
struct EntityObject : public Group
{
  explicit EntityObject(const pugi::xml_node &, Scope &);

  auto objectType() const -> ObjectType::value_type;
};

DEFINE_LAZY_VISITOR(
  const EntityObject,
  // CASE(CatalogReference),  //
  CASE(Vehicle),     //
  CASE(Pedestrian),  //
  CASE(MiscObject),  //
);

DEFINE_LAZY_VISITOR(
  EntityObject,
  // CASE(CatalogReference),  //
  CASE(Vehicle),     //
  CASE(Pedestrian),  //
  CASE(MiscObject),  //
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_OBJECT_HPP_
