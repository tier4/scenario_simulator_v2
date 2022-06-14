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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_OBJECT_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_OBJECT_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/entity_object.hpp>
#include <openscenario_interpreter/syntax/object_controller.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ScenarioObject ---------------------------------------------------------
 *
 *  <xsd:complexType name="ScenarioObject">
 *    <xsd:sequence>
 *      <xsd:group ref="EntityObject"/>
 *      <xsd:element name="ObjectController" minOccurs="0" type="ObjectController"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ScenarioObject : public Scope, public EntityObject, private SimulatorCore::GeneralCommand
{
  ObjectController object_controller;  // Controller of the EntityObject instance.

  bool is_added = false;  // NOTE: Is applied AddEntityAction?

  explicit ScenarioObject(const pugi::xml_node &, Scope &);

  auto activateOutOfRangeMetric(const Vehicle &) const -> bool;

  auto activateSensors() -> bool;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_OBJECT_HPP_
