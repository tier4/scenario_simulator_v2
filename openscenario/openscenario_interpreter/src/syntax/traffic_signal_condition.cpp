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
  if (auto && traffic_relation = getConventionalTrafficLights(boost::lexical_cast<std::int64_t>(name));
      state == "none") {
    current_state = "none";
    return asBoolean(std::all_of(
      std::begin(traffic_relation), std::end(traffic_relation),
      [](const traffic_simulator::TrafficLight & traffic_light) { return traffic_light.empty(); }));
  } else {
    std::stringstream ss;
    std::string separator = "";
    for (traffic_simulator::TrafficLight & traffic_light : traffic_relation) {
      ss << separator << traffic_light;
      separator = "; ";
    }
    current_state = ss.str();

    return asBoolean(std::all_of(
      std::begin(traffic_relation), std::end(traffic_relation),
      [this](const traffic_simulator::TrafficLight & traffic_light) {
        return traffic_light.contains(state);
      }));
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
