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
#include <openscenario_interpreter/syntax/uniform_distribution.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
UniformDistribution::UniformDistribution(
  const pugi::xml_node & node, openscenario_interpreter::Scope & scope)
: range(readElement<Range>("range", node, scope)),
  distribution(
    scope.ref<Double>(std::string("randomSeed")).data, range.lower_limit.data,
    range.upper_limit.data)
{
}

[[nodiscard]] auto UniformDistribution::evaluate() -> Object
{
  // Return a value without filtering by this->range here
  // because it is embedded in StochasticDistributionClass
  return make<Double>(distribution.generate());
}
}  // namespace syntax
}  // namespace openscenario_interpreter
