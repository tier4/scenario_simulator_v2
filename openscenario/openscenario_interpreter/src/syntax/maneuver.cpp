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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
#include <openscenario_interpreter/syntax/event.hpp>
#include <openscenario_interpreter/syntax/maneuver.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Maneuver::Maneuver(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope),
  parameter_declarations(readElement<ParameterDeclarations>("ParameterDeclarations", node, local()))
{
  traverse<1, unbounded>(node, "Event", [&](auto && node) {
    return elements.push_back(readStoryboardElement<Event>(node, local(), *this));
  });
}

auto Maneuver::run() -> void
{
  std::size_t index{0};
  for (auto && event : elements) {
    try {
      assert(event.is_also<Event>());
      event.evaluate();
      ++index;
    } catch (const SpecialAction<EXIT_FAILURE> & action) {
      throw SpecialAction<EXIT_FAILURE>(name, "Event", index, action);
    }
  }
}

auto Maneuver::overrideEvents() -> void
{
  for (auto && element : elements) {
    assert(element.is<Event>());
    element.as<Event>().override();
  }
}

auto Maneuver::running_events_count() const -> std::size_t
{
  std::size_t ret = 0;
  for (auto && element : elements) {
    assert(element.is<Event>());
    if (element.as<Event>().is<StoryboardElementState::runningState>()) {
      ++ret;
    }
  }
  return ret;
}

auto operator<<(nlohmann::json & json, const Maneuver & maneuver) -> nlohmann::json &
{
  json["name"] = maneuver.name;

  json["currentState"] = boost::lexical_cast<std::string>(maneuver.state());

  json["Event"] = nlohmann::json::array();

  for (const auto & event : maneuver.elements) {
    nlohmann::json json_event;
    json_event << event.as<Event>();
    json["Event"].push_back(json_event);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
