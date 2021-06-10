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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__OPEN_SCENARIO_CATEGORY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__OPEN_SCENARIO_CATEGORY_HPP_

#include <openscenario_interpreter/syntax/catalog_definition.hpp>
#include <openscenario_interpreter/syntax/scenario_definition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- OpenScenarioCategory ---------------------------------------------------
 *
 *  <xsd:group name="OpenScenarioCategory">
 *    <xsd:choice>
 *      <xsd:group ref="ScenarioDefinition"/>
 *      <xsd:group ref="CatalogDefinition"/>
 *    </xsd:choice>
 *  </xsd:group>
 *
 * -------------------------------------------------------------------------- */
struct OpenScenarioCategory : public Group
{
  template <typename Tree, typename... Ts>
  explicit OpenScenarioCategory(const Tree & tree, Ts &&... xs)
  // clang-format off
  : Group(
      tree.child("Catalog") ? make< CatalogDefinition>(tree, std::forward<decltype(xs)>(xs)...)
                            : make<ScenarioDefinition>(tree, std::forward<decltype(xs)>(xs)...))
  // clang-format on
  {
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__OPEN_SCENARIO_CATEGORY_HPP_
