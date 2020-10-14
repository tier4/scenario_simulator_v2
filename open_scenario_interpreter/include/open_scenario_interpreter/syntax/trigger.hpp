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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__TRIGGER_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__TRIGGER_HPP_

#include <open_scenario_interpreter/syntax/condition_group.hpp>

#include <vector>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== Trigger ==============================================================
 *
 * <xsd:complexType name="Trigger">
 *   <xsd:sequence>
 *     <xsd:element name="ConditionGroup" type="ConditionGroup" minOccurs="0" maxOccurs="unbounded"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Trigger
  : public std::vector<ConditionGroup>
{
  template<typename Node, typename Scope>
  explicit Trigger(const Node & node, Scope & scope)
  {
    callWithElements(node, "ConditionGroup", 0, unbounded, [&](auto && node)
      {
        emplace_back(node, scope);
      });
  }

  auto evaluate() const
  {
    /* -----------------------------------------------------------------------
     *
     * A trigger is then defined as an association of condition groups. A
     * trigger evaluates to true if at least one of the associated condition
     * groups evaluates to true, otherwise it evaluates to false (OR
     * operation).
     *
     * -------------------------------------------------------------------- */
    return
      asBoolean(
      std::any_of(std::begin(*this), std::end(*this), [&](auto && each)
      {
        return each.evaluate().template as<Boolean>(__FILE__, __LINE__);
      }));
  }
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__TRIGGER_HPP_
