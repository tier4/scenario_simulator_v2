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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_DEFINITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_DEFINITION_HPP_

#include <boost/json.hpp>
#include <openscenario_interpreter/syntax/catalog_locations.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/road_network.hpp>
#include <openscenario_interpreter/syntax/storyboard.hpp>
#include <pugixml.hpp>

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
  const ParameterDeclarations parameter_declarations;

  const CatalogLocations catalog_locations;

  RoadNetwork road_network;

  const Entities entities;

  Storyboard storyboard;

  explicit ScenarioDefinition(const pugi::xml_node &, Scope &);

  auto evaluate() -> Object;

  friend auto operator<<(std::ostream &, const ScenarioDefinition &) -> std::ostream &;

  friend auto operator<<(boost::json::object &, const ScenarioDefinition &)
    -> boost::json::object &;
};

}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SCENARIO_DEFINITION_HPP_
