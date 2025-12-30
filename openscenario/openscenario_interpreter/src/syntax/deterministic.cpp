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
#include <openscenario_interpreter/syntax/deterministic.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Deterministic::Deterministic(const pugi::xml_node & node, Scope & scope)
: deterministic_parameter_distributions(
    readGroups<DeterministicParameterDistribution, 0>(node, scope))
{
}

auto Deterministic::derive() -> ParameterDistribution
{
  ParameterDistribution distribution;
  for (auto additional_distribution : deterministic_parameter_distributions) {
    distribution = mergeParameterDistribution(
      distribution,
      apply<ParameterDistribution>(
        [](auto & distribution) { return distribution.derive(); }, additional_distribution));
  }
  return distribution;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
