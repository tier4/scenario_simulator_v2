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
    readAttribute<UnsignedInt>("maximumExecutionCount", node, local(), UnsignedInt(1)),
    readElement<Trigger>("StartTrigger", node, local(), Trigger({ConditionGroup(), ConditionGroup()}))),
  priority(readAttribute<Priority>("priority", node, local()))
{
  std::cout << "Start Trigger Current Value : " << start_trigger.current_value << std::endl;
  std::cout << "Start Trigger Evaluated Value : " << start_trigger.evaluate().as<Boolean>() << std::endl;
  traverse<1, unbounded>(node, "Action", [&](auto && node) {
    return elements.push_back(readStoryboardElement<Action>(node, local()));
  });
}

auto Event::start() -> void
{
  for (auto && element : elements) {
    assert(element.template is<Action>());
    assert(element.template is_also<StoryboardElement>());
    element.template as<StoryboardElement>().current_state = start_transition;
  }
}

auto operator<<(nlohmann::json & json, const Event & datum) -> nlohmann::json &
{
  json["name"] = datum.name;

  json["currentState"] = boost::lexical_cast<std::string>(datum.state());

  json["currentExecutionCount"] = datum.current_execution_count;
  json["maximumExecutionCount"] = datum.maximum_execution_count;

  json["Action"] = nlohmann::json::array();

  for (const auto & each : datum.elements) {
    nlohmann::json action;
    action << each.as<Action>();
    json["Action"].push_back(action);
  }

  json["StartTrigger"] << datum.start_trigger;

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
