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
  callWithElements(node, "CatalogReference", 0, unbounded, [&](auto && node) {
    return maneuvers.push_back(readCatalogedStoryboardElement<Maneuver>(node, local()));
  });

  callWithElements(node, "Maneuver", 0, unbounded, [&](auto && node) {
    return maneuvers.push_back(readStoryboardElement<Maneuver>(node, local()));
  });
}

auto ManeuverGroup::accomplished() const -> bool
{
  // A ManeuverGroup's goal is accomplished when all its Maneuvers are in the completeState.
  return std::all_of(std::begin(maneuvers), std::end(maneuvers), [&](auto && maneuver) {
    return maneuver.template as<Maneuver>().complete();
  });
}

auto ManeuverGroup::elements() -> Elements & { return maneuvers; }

auto ManeuverGroup::ready() noexcept -> bool { return true; }

auto ManeuverGroup::run() -> void
{
  for (auto && maneuver : maneuvers) {
    maneuver.evaluate();
  }
}

auto ManeuverGroup::start() -> void
{
  for (auto && maneuver : maneuvers) {
    maneuver.as<Maneuver>().current_state = standby_state;
  }
}

auto ManeuverGroup::stop() -> void
{
  for (auto && maneuver : maneuvers) {
    maneuver.as<Maneuver>().override();
    maneuver.evaluate();
  }
}

auto ManeuverGroup::stopTriggered() noexcept -> bool { return false; }

auto operator<<(nlohmann::json & json, const ManeuverGroup & maneuver_group) -> nlohmann::json &
{
  json["name"] = maneuver_group.name;

  json["currentState"] = boost::lexical_cast<std::string>(maneuver_group.currentState());

  json["currentExecutionCount"] = maneuver_group.current_execution_count;
  json["maximumExecutionCount"] = maneuver_group.maximum_execution_count;

  json["Maneuver"] = nlohmann::json::array();

  for (auto && maneuver : maneuver_group.maneuvers) {
    nlohmann::json json_maneuver;
    json_maneuver << maneuver.as<Maneuver>();
    json["Maneuver"].push_back(json_maneuver);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
