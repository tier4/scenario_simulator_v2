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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_HPP_

#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <pugixml.hpp>
#include <traffic_simulator_msgs/msg/driver_model.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Controller -------------------------------------------------------------
 *
 *  Defines a controller type and parameters for the controller.
 *
 *  Used in:
 *    AssignControllerAction,
 *    Catalog
 *    ControllerDistributionEntry,
 *    ObjectController,
 *
 *  <xsd:complexType name="Controller">
 *    <xsd:all>
 *      <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *      <xsd:element name="Properties" type="Properties"/>
 *    </xsd:all>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Controller : public Scope
{
  const ParameterDeclarations parameter_declarations;  // Definition of additional parameters.

  Properties properties;  // Describing properties for the controller.

  explicit Controller(const pugi::xml_node &, Scope &);

  auto assign(const EntityRef &) -> void;

  auto isUserDefinedController() & -> bool;

  auto operator[](const String &) -> const Property &;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_HPP_
