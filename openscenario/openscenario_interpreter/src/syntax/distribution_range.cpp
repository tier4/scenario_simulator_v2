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
  const auto number_of_parameters =
    static_cast<std::size_t>((range.upper_limit - range.lower_limit) / step_width + 1);
  for (std::size_t i = 0; i < number_of_parameters; ++i) {
    unnamed_distribution.emplace_back(
      make<Double>(range.lower_limit + static_cast<double>(i) * step_width));
  }
  return unnamed_distribution;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
