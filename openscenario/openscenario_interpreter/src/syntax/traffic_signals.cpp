// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/traffic_signals.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TrafficSignals::TrafficSignals(const pugi::xml_node & node, Scope & scope)
{
  for (auto & element :
       readElementsAsElement<TrafficSignalController, 0>("TrafficSignalController", node, scope)) {
    const auto controller = std::dynamic_pointer_cast<TrafficSignalController>(element);
    scope.insert(controller->name, element);
    traffic_signal_controllers.push_back(std::move(controller));
  }

  resolve_reference(scope);
}

auto TrafficSignals::evaluate() -> Object
{
  for (auto && controller : traffic_signal_controllers) {
    controller->evaluate();
  }

  return unspecified;
}

auto TrafficSignals::resolve_reference(Scope & scope) -> void
{
  for (auto & each : traffic_signal_controllers) {
    if (not each->reference.empty()) {
      try {
        auto & reference = scope.ref<TrafficSignalController>(each->reference);
        if (each->cycleTime() != reference.cycleTime()) {
          THROW_SEMANTIC_ERROR(
            "The cycle time of ", each->name, "(", each->cycleTime(), " sec) and ", each->reference,
            "(", reference.cycleTime(), " sec) is different");
        }
        reference.observers.push_back(each);
      } catch (const std::out_of_range &) {
        THROW_SYNTAX_ERROR(each->reference, " is not declared in the TrafficSignals.");
      }
    }
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
