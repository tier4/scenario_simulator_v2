// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__OPENSCENARIO_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__OPENSCENARIO_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/catalog_locations.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/file_header.hpp>
#include <openscenario_interpreter/syntax/road_network.hpp>
#include <openscenario_interpreter/syntax/storyboard.hpp>
#include <string>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ScenarioDefinition -----------------------------------------------------
 *
 * <xsd:group name="ScenarioDefinition">
 *   <xsd:sequence>
 *     <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *     <xsd:element name="CatalogLocations" type="CatalogLocations"/>
 *     <xsd:element name="RoadNetwork" type="RoadNetwork"/>
 *     <xsd:element name="Entities" type="Entities"/>
 *     <xsd:element name="Storyboard" type="Storyboard"/>
 *   </xsd:sequence>
 * </xsd:group>
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

  template <typename Node, typename Scope>
  explicit ScenarioDefinition(const Node & node, Scope & outer_scope)
  : parameter_declarations(
      readElement<ParameterDeclarations>("ParameterDeclarations", node, outer_scope)),
    catalog_locations(readElement<CatalogLocations>("CatalogLocations", node, outer_scope)),
    road_network(readElement<RoadNetwork>("RoadNetwork", node, outer_scope)),
    entities(readElement<Entities>("Entities", node, outer_scope)),
    storyboard(readElement<Storyboard>("Storyboard", node, outer_scope))
  {
  }

  template <typename... Ts>
  decltype(auto) complete(Ts &&... xs)
  {
    return storyboard.complete(std::forward<decltype(xs)>(xs)...);
  }

  template <typename... Ts>
  auto evaluate(Ts &&... xs)
  {
    road_network.evaluate();
    const auto result = storyboard.evaluate();
    updateFrame();
    return result;
  }
};

std::ostream & operator<<(std::ostream & os, const ScenarioDefinition &)
{
  return os << unspecified;
}

/* ---- OpenScenario -----------------------------------------------------------
 *
 * <xsd:complexType name="OpenScenario">
 *   <xsd:sequence>
 *     <xsd:element name="FileHeader" type="FileHeader"/>
 *     <xsd:group ref="OpenScenarioCategory"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * <xsd:group name="OpenScenarioCategory">
 *   <xsd:choice>
 *     <xsd:group ref="ScenarioDefinition"/>
 *     <xsd:group ref="CatalogDefinition"/>
 *   </xsd:choice>
 * </xsd:group>
 *
 * <xsd:group name="CatalogDefinition">
 *   <xsd:sequence>
 *     <xsd:element name="Catalog" type="Catalog"/>
 *   </xsd:sequence>
 * </xsd:group>
 *
 * -------------------------------------------------------------------------- */
struct OpenScenario : public pugi::xml_document
{
  Element category;

  Scope scope;

  const auto & load(const std::string & scenario)
  {
    const auto result{load_file(scenario.c_str())};

    if (!result) {
      std::stringstream ss{};
      ss << "while loading scenario \"" << scenario << "\" => " << result.description();
      throw SyntaxError(ss.str());
    } else {
      return *this;
    }
  }

  decltype(auto) load(const boost::filesystem::path & scenario) { return load(scenario.string()); }

  template <typename... Ts>
  explicit OpenScenario(Ts &&... xs) : scope(std::forward<decltype(xs)>(xs)...)
  {
    if (load(scope.scenario).child("OpenSCENARIO").child("Catalog")) {
      throw SyntaxError("The Catalog feature is not yet supported");
    } else {
      category = make<ScenarioDefinition>(child("OpenSCENARIO"), scope);
    }
  }

  template <typename... Ts>
  decltype(auto) complete(Ts &&... xs)
  {
    return category.as<ScenarioDefinition>().complete(std::forward<decltype(xs)>(xs)...);
  }

  template <typename... Ts>
  decltype(auto) evaluate(Ts &&... xs)
  {
    return category.evaluate(std::forward<decltype(xs)>(xs)...);
  }

  template <typename... Ts>
  decltype(auto) operator()(Ts &&... xs)
  {
    return evaluate(std::forward<decltype(xs)>(xs)...);
  }
};

template <typename... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const OpenScenario &)
{
  return os << unspecified;
}
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__OPENSCENARIO_HPP_
