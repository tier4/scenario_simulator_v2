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
    return elements.push_back(readStoryboardElement<Event>(node, local()));
  });
}

Maneuver::Maneuver(
  const pugi::xml_node & node, Scope & scope, const ParameterAssignments & parameter_assignments)
: Scope(readAttribute<String>("name", node, scope), scope, parameter_assignments),
  parameter_declarations(readElement<ParameterDeclarations>("ParameterDeclarations", node, local()))
{
  traverse<1, unbounded>(node, "Event", [&](auto && node) {
    return elements.push_back(readStoryboardElement<Event>(node, local()));
  });
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
