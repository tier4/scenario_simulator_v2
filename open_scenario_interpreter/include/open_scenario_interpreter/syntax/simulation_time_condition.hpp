// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__SIMULATION_TIME_CONDITION_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__SIMULATION_TIME_CONDITION_HPP_

#include <open_scenario_interpreter/syntax/rule.hpp>

#include <chrono>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== SimulationTimeCondition ==============================================
 *
 * <xsd:complexType name="SimulationTimeCondition">
 *   <xsd:attribute name="value" type="Double" use="required"/>
 *   <xsd:attribute name="rule" type="Rule" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct SimulationTimeCondition
{
  const Double value;

  const Rule compare;

  std::chrono::high_resolution_clock::time_point begin;

  template<typename Node, typename Scope>
  explicit SimulationTimeCondition(const Node & node, Scope & scope)
  : value(readAttribute<Double>("value", node, scope)),
    compare(readAttribute<Rule>("rule", node, scope)),
    begin(std::chrono::high_resolution_clock::now())  // XXX TEMPORARY TOY IMPLEMENTATION
  {}

  auto evaluate() const
  {
    const auto simulation_time {
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now() - begin
      ).count()
    };

    const auto specified_time {
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::seconds(value)
      ).count()
    };

    const auto result {compare(simulation_time, specified_time) ? true_v : false_v};

    std::cout << indent << "SimulationTime [" << simulation_time << " is " << compare << " " <<
      specified_time << "? => " << result << "]" << std::endl;

    return result;
  }
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__SIMULATION_TIME_CONDITION_HPP_
