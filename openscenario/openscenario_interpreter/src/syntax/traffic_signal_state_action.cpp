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
#include <openscenario_interpreter/syntax/traffic_signal_state_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TrafficSignalStateAction::TrafficSignalStateAction(const pugi::xml_node & node, Scope & scope)
: name(readAttribute<String>("name", node, scope)),
  state(readAttribute<String>("state", node, scope))
{
}

auto TrafficSignalStateAction::accomplished() noexcept -> bool { return true; }

auto TrafficSignalStateAction::endsImmediately() noexcept -> bool { return true; }

auto TrafficSignalStateAction::run() noexcept -> void {}

auto TrafficSignalStateAction::start() const -> void
{
  setConventionalTrafficLightsState(id(), state);
}

auto TrafficSignalStateAction::id() const -> std::int64_t
{
  return boost::lexical_cast<std::int64_t>(name);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
