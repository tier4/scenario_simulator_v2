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
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
#include <openscenario_interpreter/syntax/maneuver_group.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ManeuverGroup::ManeuverGroup(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope),
  StoryboardElement(
    readAttribute<UnsignedInteger>("maximumExecutionCount", node, local(), UnsignedInteger())),
  actors(readElement<Actors>("Actors", node, local()))
{
  traverse<0, unbounded>(node, "CatalogReference", [&](auto && node) {
    return elements.push_back(readCatalogedStoryboardElement<Maneuver>(node, local()));
  });

  traverse<0, unbounded>(node, "Maneuver", [&](auto && node) {
    return elements.push_back(readStoryboardElement<Maneuver>(node, local()));
  });
}

auto ManeuverGroup::run() -> void
{
  std::size_t index{0};
  for (auto && maneuver : elements) {
    try {
      assert(maneuver.is_also<Maneuver>());
      maneuver.evaluate();
      ++index;
    } catch (const SpecialAction<EXIT_FAILURE> & action) {
      throw SpecialAction<EXIT_FAILURE>(name, "Maneuver", index, action);
    }
  }
}

auto ManeuverGroup::start() -> void
{
  for (auto && element : elements) {
    assert(element.template is<Maneuver>());
    assert(element.template is_also<StoryboardElement>());
    element.template as<StoryboardElement>().transitionTo(start_transition);
  }
}

auto operator<<(nlohmann::json & json, const ManeuverGroup & maneuver_group) -> nlohmann::json &
{
  json["name"] = maneuver_group.name;

  json["currentState"] = boost::lexical_cast<std::string>(maneuver_group.state());

  json["currentExecutionCount"] = maneuver_group.current_execution_count;
  json["maximumExecutionCount"] = maneuver_group.maximum_execution_count;

  json["Maneuver"] = nlohmann::json::array();

  for (auto && maneuver : maneuver_group.elements) {
    nlohmann::json json_maneuver;
    json_maneuver << maneuver.as<Maneuver>();
    json["Maneuver"].push_back(json_maneuver);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
