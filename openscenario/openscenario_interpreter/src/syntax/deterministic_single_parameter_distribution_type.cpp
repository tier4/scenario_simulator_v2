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
#include <openscenario_interpreter/syntax/deterministic_single_parameter_distribution_type.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
DeterministicSingleParameterDistributionType::DeterministicSingleParameterDistributionType(
  const pugi::xml_node & tree, Scope & scope)
// clang-format off
: Group(
    choice(tree,
      {"DistributionSet",         [&](auto && node){ return make<DistributionSet        >(node, scope);}},
      {"DistributionRange",       [&](auto && node){ return make<DistributionRange      >(node, scope);}}))
//      {"UserDefinedDistribution", [&](auto && node){ return make<UserDefinedDistribution>(node, scope);}}))
// clang-format on
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
