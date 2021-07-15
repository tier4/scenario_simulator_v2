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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_ACTION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- NOTE -------------------------------------------------------------------
 *
 *  Sets a specific phase of a traffic signal controller, typically affecting a
 *  collection of signals.
 *
 *  <xsd:complexType name="TrafficSignalControllerAction">
 *    <xsd:attribute name="trafficSignalControllerRef" type="String" use="required"/>
 *    <xsd:attribute name="phase" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct TrafficSignalControllerAction : private Scope
{
  /* ---- NOTE -----------------------------------------------------------------
   *
   *  ID of the signal controller in a road network.
   *
   * ------------------------------------------------------------------------ */
  const String traffic_signal_controller_ref;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  Targeted phase of the signal controller. The available phases are defined
   *  in type RoadNetwork under the property trafficSignalControllers.
   *
   * ------------------------------------------------------------------------ */
  const String phase;

  template <typename Node>
  explicit TrafficSignalControllerAction(const Node & node, const Scope & current_scope)
  : Scope(current_scope),
    traffic_signal_controller_ref(
      readAttribute<String>("trafficSignalControllerRef", node, localScope())),
    phase(readAttribute<String>("phase", node, localScope()))
  {
  }

  static auto accomplished() noexcept -> bool { return true; }

  auto start()
  {
    try {
      auto & controller = localScope().traffic_signal_controllers.at(traffic_signal_controller_ref);
      (*controller).changePhaseByName(phase);
      return unspecified;
    } catch (const std::out_of_range &) {
      THROW_SYNTAX_ERROR(
        "TrafficSignalController ", std::quoted(traffic_signal_controller_ref),
        " is not declared in this scope");
    }
  }

  static auto endsImmediately() -> bool { return true; }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_ACTION_HPP_
