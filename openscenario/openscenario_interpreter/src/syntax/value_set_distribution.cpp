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
#include <openscenario_interpreter/syntax/value_set_distribution.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ValueSetDistribution::ValueSetDistribution(
  const pugi::xml_node & node, openscenario_interpreter::Scope & scope)
: Scope(scope),
  parameter_value_sets(readElements<ParameterValueSet, 1>("ParameterValueSet", node, scope))
{
}

auto ValueSetDistribution::derive() -> ParameterDistribution
{
  ParameterDistribution parameters;
  for (const auto & parameter_value_set : parameter_value_sets) {
    parameters.emplace_back(parameter_value_set.evaluate());
  }
  return parameters;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
