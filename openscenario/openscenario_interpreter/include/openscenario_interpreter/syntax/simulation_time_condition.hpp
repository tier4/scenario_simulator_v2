// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SIMULATION_TIME_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SIMULATION_TIME_CONDITION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- SimulationTimeCondition ------------------------------------------------
 *
 *  <xsd:complexType name="SimulationTimeCondition">
 *    <xsd:attribute name="value" type="Double" use="required"/>
 *    <xsd:attribute name="rule" type="Rule" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct SimulationTimeCondition
{
  const Double value;

  const Rule compare;

  template<typename Node, typename Scope>
  explicit SimulationTimeCondition(const Node & node, Scope & scope)
  : value(readAttribute<Double>("value", node, scope)),
    compare(readAttribute<Rule>("rule", node, scope))
  {}

  Element result;

  auto evaluate()
  {
    result = asBoolean(compare(getCurrentTime(), value));
    #ifndef NDEBUG
    std::cout << *this << std::endl;
    #endif
    return result;
  }

  friend std::ostream & operator<<(std::ostream & os, const SimulationTimeCondition & condition)
  {
    os << indent;
    os << "- BVC.STC: ";
    os << getCurrentTime();
    os << " is ";
    os << condition.compare;
    os << " ";
    os << condition.value;
    os << "? => ";
    os << condition.result;
    return os;
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SIMULATION_TIME_CONDITION_HPP_
