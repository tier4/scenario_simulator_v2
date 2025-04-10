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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_condition.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_state.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TrafficSignalCondition::TrafficSignalCondition(const pugi::xml_node & node, Scope & scope)
: name(readAttribute<String>("name", node, scope)),
  state(readAttribute<String>("state", node, scope))
{
}

auto TrafficSignalCondition::description() const -> String
{
  std::stringstream description;

  description << "Is TrafficSignal " << std::quoted(name) << " (" << current_state << ") in state "
              << std::quoted(state) << "?";

  return description.str();
}

auto TrafficSignalCondition::evaluate() -> Object
{
  const auto lanelet_id = boost::lexical_cast<std::int64_t>(name);
  current_state = getConventionalTrafficLightsComposedState(lanelet_id);
  if (current_state.empty()) {
    current_state = "none";
  }
  return asBoolean(compareConventionalTrafficLightsState(lanelet_id, state));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
