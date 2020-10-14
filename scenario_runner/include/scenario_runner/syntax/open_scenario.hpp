// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__SYNTAX__OPEN_SCENARIO_HPP_
#define SCENARIO_RUNNER__SYNTAX__OPEN_SCENARIO_HPP_

#include <scenario_runner/syntax/catalog_locations.hpp>
#include <scenario_runner/syntax/entities.hpp>
#include <scenario_runner/syntax/file_header.hpp>
#include <scenario_runner/syntax/road_network.hpp>
#include <scenario_runner/syntax/storyboard.hpp>

#include <string>
#include <utility>
#include <vector>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== ScenarioDefinition ===================================================
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
 * ======================================================================== */
struct ScenarioDefinition
{
  Element storyboard;

  Scope inner_scope;

  template<typename Node, typename Scope>
  explicit ScenarioDefinition(const Node & node, Scope & outer_scope)
  : inner_scope{outer_scope}
  {
    std::cout << (indent++) << "<OpenSCENARIO>" << std::endl;

    callWithElements(
      node, "ParameterDeclarations", 0, unbounded, [&](auto && each)
      {
        return make<ParameterDeclarations>(each, inner_scope);
      });

    for (const auto & each : inner_scope.parameters) {
      std::cout << indent << "<!-- Parameter " << cyan << "\'" << std::get<0>(each) << "\'" <<
        reset << " of type " << green << std::get<1>(each).type().name() << reset <<
        " declared as value " << cyan << "\"" << std::get<1>(each) << cyan << "\"" << reset <<
        " -->" << std::endl;
    }

    callWithElements(
      node, "CatalogLocations", 0, 1, [&](auto && node)
      {
        return make<CatalogLocations>(node, inner_scope);
      });

    callWithElements(
      node, "RoadNetwork", 1, 1, [&](auto && node)
      {
        return make<RoadNetwork>(node, inner_scope);
      });

    callWithElement(
      node, "Entities", [&](auto && node)
      {
        return make<Entities>(node, inner_scope);
      });

    std::cout << (indent++) << "<Entities>" << std::endl;

    for (const auto & each : inner_scope.entities) {
      std::cout << std::get<1>(each) << std::endl;
    }

    std::cout << (--indent) << "</Entities>" << std::endl;

    callWithElement(
      node, "Storyboard", [&](auto && node)
      {
        return storyboard = make<Storyboard>(node, inner_scope);
      });

    std::cout << (--indent) << "</OpenSCENARIO>" << std::endl;
  }

  template<typename ... Ts>
  auto init(Ts && ... xs)  // RENAME TO 'start'
  {
    // if (inner_scope.connection and inner_scope.connection->simulation)
    // {
    //   inner_scope.connection->simulation->initialize(1.0, 0.02);
    // }
    // else
    // {
    //   THROW(ConnectionError);
    // }

    for (const auto & each : inner_scope.entities) {
      std::cout << std::get<1>(each).evaluate() << std::endl;
    }
  }

  template<typename ... Ts>
  decltype(auto) complete(Ts && ... xs)
  {
    return storyboard.as<Storyboard>().complete(std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  auto evaluate(Ts && ... xs)
  {
    const auto result {storyboard.evaluate()};

    // inner_scope.connection->simulation->updateFrame();

    return result;
  }
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const ScenarioDefinition &)
{
  return os << unspecified;
}

/* ==== OpenScenario =========================================================
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
 * ======================================================================== */
struct OpenScenario
  : public pugi::xml_document
{
  Element category;

  Scope global;

  decltype(auto) load(const std::string & scenario)
  {
    // std::cout << "\x1b[32mLoading scenario \x1b[36m\"" << scenario << "\"\x1b[32m => \x1b[0m";

    const auto result {load_file(scenario.c_str())};

    if (!result) {
      std::stringstream ss {};
      ss << "while loading scenario \"" << scenario << "\" => " << result.description();
      throw SyntaxError {ss.str()};
    } else {
      return *this;
    }
  }

  template<typename ... Ts>
  explicit OpenScenario(const std::string & scenario, Ts && ... xs)
  : pugi::xml_document(), global{std::forward<decltype(xs)>(xs)...}
  {
    global.scenario = scenario;

    if (load(scenario).child("OpenSCENARIO").child("Catalog")) {
      THROW_IMPLEMENTATION_FAULT();
    } else {
      category = make<ScenarioDefinition>(child("OpenSCENARIO"), global);
    }
  }

  template<typename ... Ts>
  decltype(auto) init(Ts && ... xs)
  {
    if (category.is<ScenarioDefinition>()) {
      return category.as<ScenarioDefinition>().init(std::forward<decltype(xs)>(xs)...);
    } else {
      THROW_IMPLEMENTATION_FAULT();
    }
  }

  template<typename ... Ts>
  decltype(auto) complete(Ts && ... xs)
  {
    if (category.is<ScenarioDefinition>()) {
      return category.as<ScenarioDefinition>().complete(std::forward<decltype(xs)>(xs)...);
    } else {
      THROW_IMPLEMENTATION_FAULT();
    }
  }

  template<typename ... Ts>
  decltype(auto) evaluate(Ts && ... xs)
  {
    return category.evaluate(std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) operator()(Ts && ... xs)
  {
    return evaluate(std::forward<decltype(xs)>(xs)...);
  }
};

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const OpenScenario &)
{
  return os << unspecified;
}
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__OPEN_SCENARIO_HPP_
