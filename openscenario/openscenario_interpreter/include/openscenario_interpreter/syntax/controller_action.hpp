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

#include <openscenario_interpreter/syntax/assign_controller_action.hpp>
#include <openscenario_interpreter/syntax/override_controller_value_action.hpp>
#include <type_traits>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ControllerAction -------------------------------------------------------
 *
 *  Action that assigns a new controller or overrides an existing one.
 *
 *  <xsd:complexType name="ControllerAction">
 *    <xsd:all>
 *      <xsd:element name="AssignControllerAction" type="AssignControllerAction"/>
 *      <xsd:element name="OverrideControllerValueAction" type="OverrideControllerValueAction"/>
 *    </xsd:all>
 *  </xsd:complexType>
 *
 * ------------------------------------------------------------------------ */
struct ControllerAction
{
  const AssignControllerAction assign_controller_action;  // Assign a controller to an entity.

  const OverrideControllerValueAction override_controller_value_action;
  // Values for throttle, brake, clutch, parking brake, steering wheel or gear.

  template <typename Node, typename Scope>
  explicit ControllerAction(const Node & node, Scope & outer_scope)
  : assign_controller_action(
      readElement<AssignControllerAction>("AssignControllerAction", node, outer_scope)),
    override_controller_value_action(readElement<OverrideControllerValueAction>(
      "OverrideControllerValueAction", node, outer_scope))  // NOTE: DUMMY IMPLEMENTATION
  {
  }

  static auto accomplished() noexcept { return true; }

  static auto endsImmediately() noexcept { return true; };

  auto run() const -> void;

  static auto start() noexcept {}
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_ACTION_HPP_
