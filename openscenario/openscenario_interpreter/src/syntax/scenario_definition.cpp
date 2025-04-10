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
#include <openscenario_interpreter/syntax/scenario_definition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ScenarioDefinition::ScenarioDefinition(const pugi::xml_node & node, Scope & scope)
: parameter_declarations(readElement<ParameterDeclarations>("ParameterDeclarations", node, scope)),
  catalog_locations(readElement<CatalogLocations>("CatalogLocations", node, scope)),
  road_network(readElement<RoadNetwork>("RoadNetwork", node, scope)),
  entities(readElement<Entities>("Entities", node, scope)),
  storyboard(readElement<Storyboard>("Storyboard", node, scope))
{
}

auto ScenarioDefinition::evaluate() -> Object
{
  road_network.evaluate();
  try {
    return storyboard.evaluate();
  } catch (const SpecialAction<EXIT_FAILURE> & action) {
    throw SpecialAction<EXIT_FAILURE>("OpenSCENARIO", action);
  }
}

auto operator<<(std::ostream & os, const ScenarioDefinition & datum) -> std::ostream &
{
  boost::json::object json;

  return os << (json << datum);
}

auto operator<<(boost::json::object & json, const ScenarioDefinition & datum)
  -> boost::json::object &
{
  json["Storyboard"].emplace_object() << datum.storyboard;

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
