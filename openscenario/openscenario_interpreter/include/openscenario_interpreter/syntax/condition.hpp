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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_HPP_

#include <nlohmann/json.hpp>
#include <openscenario_interpreter/syntax/by_entity_condition.hpp>
#include <openscenario_interpreter/syntax/by_value_condition.hpp>
#include <openscenario_interpreter/syntax/condition_edge.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Condition --------------------------------------------------------------
 *
 *  <xsd:complexType name="Condition">
 *    <xsd:choice>
 *      <xsd:element name="ByEntityCondition" type="ByEntityCondition"/>
 *      <xsd:element name="ByValueCondition" type="ByValueCondition"/>
 *    </xsd:choice>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="delay" type="Double" use="required"/>
 *    <xsd:attribute name="conditionEdge" type="ConditionEdge" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Condition : public Element
{
  const String name;

  const Double delay;

  const ConditionEdge condition_edge;

  Boolean current_evaluation;

  template <typename Node, typename Scope>
  explicit Condition(const Node & node, Scope & scope)
  // clang-format off
  : Element(
      choice(node,
        std::make_pair("ByEntityCondition", [&](auto && node) { return make<ByEntityCondition>(node, scope); }),
        std::make_pair( "ByValueCondition", [&](auto && node) { return make< ByValueCondition>(node, scope); }))),
    name(readAttribute<String>("name", node, scope)),
    delay(readAttribute<Double>("delay", node, scope, Double())),
    condition_edge(readAttribute<ConditionEdge>("conditionEdge", node, scope)),
    current_evaluation(false)
  // clang-format on
  {
  }

  const auto & evaluate()
  {
    if (condition_edge == ConditionEdge::sticky and current_evaluation) {
      return true_v;
    } else {
      return asBoolean(current_evaluation = Element::evaluate().as<Boolean>());
    }
  }
};

nlohmann::json & operator<<(nlohmann::json &, const Condition &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_HPP_
