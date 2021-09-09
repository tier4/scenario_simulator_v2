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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_DEFINITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_DEFINITION_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/syntax/catalog_locations.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/road_network.hpp>
#include <openscenario_interpreter/syntax/storyboard.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ScenarioDefinition -----------------------------------------------------
 *
 *  <xsd:group name="ScenarioDefinition">
 *    <xsd:sequence>
 *      <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *      <xsd:element name="CatalogLocations" type="CatalogLocations"/>
 *      <xsd:element name="RoadNetwork" type="RoadNetwork"/>
 *      <xsd:element name="Entities" type="Entities"/>
 *      <xsd:element name="Storyboard" type="Storyboard"/>
 *    </xsd:sequence>
 *  </xsd:group>
 *
 * -------------------------------------------------------------------------- */
struct ScenarioDefinition
{
  ASSERT_IS_OPTIONAL_ELEMENT(ParameterDeclarations);
  const ParameterDeclarations parameter_declarations;

  const CatalogLocations catalog_locations;

  RoadNetwork road_network;

  const Entities entities;

  Storyboard storyboard;

  template <typename Node>
  explicit ScenarioDefinition(const Node & node, Scope & outer_scope)
  : parameter_declarations(
      readElement<ParameterDeclarations>("ParameterDeclarations", node, outer_scope)),
    catalog_locations(readElement<CatalogLocations>("CatalogLocations", node, outer_scope)),
    road_network(readElement<RoadNetwork>("RoadNetwork", node, outer_scope)),
    entities(readElement<Entities>("Entities", node, outer_scope)),
    storyboard(readElement<Storyboard>("Storyboard", node, outer_scope))
  {
  }

  auto complete() { return storyboard.complete(); }

  auto evaluate()
  {
    road_network.evaluate();
    storyboard.evaluate();
    updateFrame();
    return storyboard.current_state;
  }
};

std::ostream & operator<<(std::ostream &, const ScenarioDefinition &);

nlohmann::json & operator<<(nlohmann::json &, const ScenarioDefinition &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_DEFINITION_HPP_
