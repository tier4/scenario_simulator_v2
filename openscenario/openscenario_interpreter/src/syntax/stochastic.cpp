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
#include <openscenario_interpreter/syntax/stochastic.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/**
   Note: Stochastic.randomSeed is initialized with 0, if it is not specified in
   scenario. The behavior related to this implementation is not specified in
   the OpenSCENARIO standard, so it may change in the future.
 */
Stochastic::Stochastic(const pugi::xml_node & node, Scope & scope)
: number_of_test_runs(readAttribute<UnsignedInt>("numberOfTestRuns", node, scope)),
  random_seed(
    scope.seed = static_cast<double>(readAttribute<Double>("randomSeed", node, scope, 0))),
  stochastic_distribution(
    readElement<StochasticDistribution>("StochasticDistribution", node, scope))
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
