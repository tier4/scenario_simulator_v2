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
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/event.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Event::Event(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope),
  StoryboardElement(
    readAttribute<UnsignedInt>("maximumExecutionCount", node, local(), UnsignedInt(1))),
  priority(readAttribute<Priority>("priority", node, local())),
  start_trigger(readElement<Trigger>("StartTrigger", node, local()))
{
  callWithElements(node, "Action", 1, unbounded, [&](auto && node) {
    return actions.push_back(readStoryboardElement<Action>(node, local()));
  });
}

auto Event::accomplished() const -> bool
{
  // An Event's goal is accomplished when all its Actions are in the completeState.
  return std::all_of(std::begin(actions), std::end(actions), [](auto && action) {
    return action.template as<Action>().complete();
  });
}

auto Event::elements() -> Elements & { return actions; }

auto Event::ready() -> bool { return start_trigger.evaluate().as<Boolean>(); }

auto Event::run() -> void
{
  for (auto && action : actions) {
    action.evaluate();
  }
}

auto Event::start() -> void
{
  for (auto && each : actions) {
    each.as<Action>().current_state = standby_state;
  }
}

auto Event::stop() -> void
{
  for (auto && each : actions) {
    each.as<Action>().override();
    each.evaluate();
  }
}

auto Event::stopTriggered() noexcept -> bool { return false; }

auto operator<<(nlohmann::json & json, const Event & datum) -> nlohmann::json &
{
  json["name"] = datum.name;

  json["currentState"] = boost::lexical_cast<std::string>(datum.currentState());

  json["currentExecutionCount"] = datum.current_execution_count;
  json["maximumExecutionCount"] = datum.maximum_execution_count;

  json["Action"] = nlohmann::json::array();

  for (const auto & each : datum.actions) {
    nlohmann::json action;
    action << each.as<Action>();
    json["Action"].push_back(action);
  }

  json["StartTrigger"] << datum.start_trigger;

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
