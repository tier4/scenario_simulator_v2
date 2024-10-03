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

#define FMT_HEADER_ONLY

#include <fmt/format.h>

#include <iomanip>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/simulation_time_condition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
SimulationTimeCondition::SimulationTimeCondition(const pugi::xml_node & node, Scope & scope)
: value(readAttribute<Double>("value", node, scope)),
  compare(readAttribute<Rule>("rule", node, scope))
{
  std::stringstream os;
  os << "is " << compare << " " << value << "?";
  description_condition_part = os.str();
}

auto SimulationTimeCondition::description() const -> String
{
  return fmt::format(
    "Is the simulation time (= {:.30f}) {}", static_cast<double>(result),
    description_condition_part);
}

auto SimulationTimeCondition::evaluate() -> Object
{
  return asBoolean(compare(result = evaluateSimulationTime(), value));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
