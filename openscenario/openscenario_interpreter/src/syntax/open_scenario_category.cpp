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

#ifndef PARAMETER_VALUE_DISTRIBUTION_ONLY
#include <openscenario_interpreter/syntax/catalog_definition.hpp>
#include <openscenario_interpreter/syntax/scenario_definition.hpp>
#endif  // PARAMETER_VALUE_DISTRIBUTION_ONLY

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/open_scenario_category.hpp>
#include <openscenario_interpreter/syntax/parameter_value_distribution_definition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
OpenScenarioCategory::OpenScenarioCategory(const pugi::xml_node & tree, Scope & scope)
: Group(choice(
    tree, {
#ifndef PARAMETER_VALUE_DISTRIBUTION_ONLY
            {"Storyboard",
             [&](auto &&) { return make<ScenarioDefinition>(tree, scope); }},  // DIRTY HACK!!!
            {"Catalog", [&](auto &&) { return make<CatalogDefinition>(tree, scope); }},
#endif  // PARAMETER_VALUE_DISTRIBUTION_ONLY
            {"ParameterValueDistribution",
             [&](auto &&) { return make<ParameterValueDistributionDefinition>(tree, scope); }}}))
// clang-format on
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
