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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/assign_controller_action.hpp>
#include <openscenario_interpreter/syntax/override_controller_value_action.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ControllerAction (OSC 1.1) ---------------------------------------------
 *
 *  Action that assigns a new controller or overrides an existing one.
 *
 *  <xsd:complexType name="ControllerAction">
 *    <xsd:all>
 *      <xsd:element name="AssignControllerAction" type="AssignControllerAction" minOccurs="0"/>
 *      <xsd:element name="OverrideControllerValueAction" type="OverrideControllerValueAction" minOccurs="0"/>
 *      <xsd:element name="ActivateControllerAction" type="ActivateControllerAction" minOccurs="0"/>
 *    </xsd:all>
 *  </xsd:complexType>
 *
 * ------------------------------------------------------------------------ */
struct ControllerAction : private Scope
{
  // Assign a controller to an entity.
  const AssignControllerAction assign_controller_action;

  // Values for throttle, brake, clutch, parking brake, steering wheel or gear.
  const OverrideControllerValueAction override_controller_value_action;

  explicit ControllerAction(const pugi::xml_node &, Scope &);

  static auto accomplished() noexcept -> bool;

  static auto endsImmediately() noexcept -> bool;

  static auto run() noexcept -> void;

  /*  */ auto start() const -> void;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_ACTION_HPP_
