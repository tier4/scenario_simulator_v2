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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNALS_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNALS_HPP_

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller.hpp>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- TrafficSignals ---------------------------------------------------------
 *
 *  <xsd:complexType name="TrafficSignals">
 *    <xsd:sequence>
 *      <xsd:element name="TrafficSignalController" minOccurs="0" maxOccurs="unbounded" type="TrafficSignalController"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct TrafficSignals
{
  std::list<TrafficSignalController> traffic_signal_controllers;

  TrafficSignals() = default;

  template <typename Node, typename Scope>
  explicit TrafficSignals(const Node & node, Scope & outer_scope)
  : traffic_signal_controllers(
      readElements<TrafficSignalController, 0>("TrafficSignalController", node, outer_scope))
  {
    for (auto & each : traffic_signal_controllers) {
      const auto result = outer_scope.traffic_signal_controller_refs.emplace(each.name, each);
      if (not result.second) {
        throw SyntaxError(
          "Multiple TrafficSignalControllers have been declared with the same name: ", each.name);
      }
    }
  }

  auto evaluate()
  {
    for (auto && controller : traffic_signal_controllers) {
      controller.evaluate();
    }

    return unspecified;
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNALS_HPP_
