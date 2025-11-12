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
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/phase.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Phase::Phase(const pugi::xml_node & node, Scope & scope)
: name(readAttribute<String>("name", node, scope)),
  duration(readAttribute<Double>("duration", node, scope, Double::infinity())),
  traffic_signal_states(readElements<TrafficSignalState, 0>("TrafficSignalState", node, scope)),
  grouped_states([this]() {
    std::map<lanelet::Id, std::vector<const TrafficSignalState *>> groups;
    for (const auto & traffic_signal_state : traffic_signal_states) {
      groups[traffic_signal_state.id()].push_back(&traffic_signal_state);
    }
    return groups;
  }())
{
}

auto Phase::evaluate() const -> Object
{
  for (auto && [id, states] : grouped_states) {
    for (size_t i = 0; i < states.size(); ++i) {
      states[i]->evaluate();
    }
  }

  return unspecified;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
