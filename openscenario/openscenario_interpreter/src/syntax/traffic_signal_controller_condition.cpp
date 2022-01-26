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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller_condition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TrafficSignalControllerCondition::TrafficSignalControllerCondition(
  const pugi::xml_node & tree, const Scope & scope)
: phase(readAttribute<String>("phase", tree, scope)),
  traffic_signal_controller_ref(readAttribute<String>("trafficSignalControllerRef", tree, scope)),
  scope(scope)
{
}

auto TrafficSignalControllerCondition::description() const -> String
{
  std::stringstream description;

  description << "Is controller " << std::quoted(traffic_signal_controller_ref)  //
              << " (Phase = "                                                    //
              << current_phase_name                                              //
              << ", since " << current_phase_since                               //
              << " sec) in phase " << std::quoted(phase) << "?";

  return description.str();
}

auto TrafficSignalControllerCondition::evaluate() -> Object
{
  const auto & controller = scope.ref<TrafficSignalController>(traffic_signal_controller_ref);
  current_phase_name = controller.currentPhaseName();
  current_phase_since = controller.currentPhaseSince();
  return asBoolean(current_phase_name == phase);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
