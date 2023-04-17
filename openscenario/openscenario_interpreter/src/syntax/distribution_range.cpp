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
#include <openscenario_interpreter/syntax/distribution_range.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
DistributionRange::DistributionRange(const pugi::xml_node & node, Scope & scope)
: Scope(scope),
  step_width(readAttribute<Double>("stepWidth", node, local())),
  range(readElement<Range>("Range", node, local()))
{
}

auto DistributionRange::derive() -> SingleUnnamedParameterDistribution
{
  SingleUnnamedParameterDistribution unnamed_distribution;
  for (double parameter = range.lower_limit; parameter <= range.upper_limit;
       parameter += step_width) {
    unnamed_distribution.emplace_back(make<Double>(parameter));
  }
  return unnamed_distribution;
}

auto DistributionRange::derive(
  std::size_t local_index, std::size_t local_size, std::size_t global_index,
  std::size_t global_size) -> ParameterList
{
  return ParameterList({{"", make<Double>(range.lower_limit + step_width * local_index)}});
}

auto DistributionRange::getNumberOfDeriveScenarios() const -> std::size_t
{
  return int((range.upper_limit - range.lower_limit) / step_width) + 1;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
