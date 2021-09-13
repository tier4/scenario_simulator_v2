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

#include <cassert>
#include <memory>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller.hpp>
#include <openscenario_interpreter/utility/circular_check.hpp>

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
private:
  std::list<std::shared_ptr<TrafficSignalController>> traffic_signal_controllers;

public:
  TrafficSignals() = default;

  template <typename Node>
  explicit TrafficSignals(const Node & node, Scope & outer_scope)
  {
    for (auto & element : readElementsAsElement<TrafficSignalController, 0>(
           "TrafficSignalController", node, outer_scope)) {
      const auto controller = std::dynamic_pointer_cast<TrafficSignalController>(element);
      outer_scope.insert(controller->name, element);
      traffic_signal_controllers.push_back(std::move(controller));
    }

    resolve_reference(outer_scope);
  }

  auto evaluate()
  {
    for (auto && controller : traffic_signal_controllers) {
      controller->evaluate();
    }

    return unspecified;
  }

private:
  void resolve_reference(Scope & scope)
  {
    for (auto & each : traffic_signal_controllers) {
      if (!each->reference.empty()) {
        try {
          auto & reference = scope.findElement(each->reference).as<TrafficSignalController>();
          if (each->cycleTime() != reference.cycleTime()) {
            THROW_SEMANTIC_ERROR(
              "The cycle time of ", each->name, "(", each->cycleTime(), " sec) and ",
              each->reference, "(", reference.cycleTime(), " sec) is different");
          }
          reference.observers.push_back(each);
        } catch (std::out_of_range &) {
          THROW_SYNTAX_ERROR(each->reference, "is not declared in the TrafficSignals.");
        }
      }
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNALS_HPP_
