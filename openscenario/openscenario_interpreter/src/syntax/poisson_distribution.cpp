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
#include <openscenario_interpreter/syntax/poisson_distribution.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
PoissonDistribution::PoissonDistribution(
  const pugi::xml_node & node, openscenario_interpreter::Scope & scope)
: Scope(scope),
  range(readElement<Range>("Range", node, scope)),
  expected_value(readAttribute<Double>("expectedValue", node, scope)),
  distribute(expected_value.data)
{
}

auto PoissonDistribution::derive() -> Object
{
  return make<Double>([&]() {
    double value;
    auto in_range = [this](double value) {
      return range.lower_limit.data <= value and value <= range.upper_limit.data;
    };
    do {
      value = distribute(random_engine);
    } while (not in_range(value));
    return value;
  }());
}
}  // namespace syntax
}  // namespace openscenario_interpreter
