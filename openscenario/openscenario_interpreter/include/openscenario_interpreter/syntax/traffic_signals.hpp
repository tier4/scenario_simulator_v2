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

#include <memory>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller.hpp>
#include <openscenario_interpreter/utility/circular_check.hpp>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "openscenario_interpreter/utility/circular_check.hpp"
#include "scenario_simulator_exception/exception.hpp"

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

  using NameToController =
    std::unordered_map<std::string, std::shared_ptr<TrafficSignalController>>;

  void resolve_reference(const NameToController & controllers)
  {
    std::unordered_map<std::string, std::vector<std::string>> reference_map;
    for (auto & each : traffic_signal_controllers) {
      if (!each->reference.empty()) {
        try {
          auto reference = controllers.at(each->reference);
          each->reference_controller = reference;
        } catch (std::out_of_range &) {
          THROW_SYNTAX_ERROR(each->reference, "is not declared in the TrafficSignals.");
        }
      }
    }

    for (auto & controller : traffic_signal_controllers) {
      if (controller) {
        bool is_circular = utility::circular_check(
          controller->name, [&reference_map](const std::string & node) -> decltype(auto) {
            return reference_map[node];
          });

        if (is_circular) {
          THROW_SEMANTIC_ERROR("detect circular reference among TrafficSignalControlers");
        }
      }
    }
  }

public:
  TrafficSignals() = default;

  template <typename Node>
  explicit TrafficSignals(const Node & node, Scope & outer_scope)
  : traffic_signal_controllers(
      readSharedElements<TrafficSignalController, 0>("TrafficSignalController", node, outer_scope))
  {
    for (auto && each : traffic_signal_controllers) {
      if (each) {
        auto result = outer_scope.traffic_signal_controllers.emplace(each->name, each);
        if (not result.second) {
          throw SyntaxError(
            "Multiple TrafficSignalControllers have been declared with the same name: ",
            each->name);
        }
      }
    }

    resolve_reference(outer_scope.traffic_signal_controllers);
  }

  auto evaluate()
  {
    for (auto && controller : traffic_signal_controllers) {
      controller->evaluate();
    }

    return unspecified;
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNALS_HPP_
