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
#include <openscenario_interpreter/syntax/file_header.hpp>
#include <openscenario_interpreter/syntax/scenario_definition.hpp>
#include <string>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
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
    const auto result = load_file(scenario.c_str());

    if (!result) {
      throw SyntaxError(
        "while loading scenario ", std::quoted(scenario), " => ", result.description());
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
