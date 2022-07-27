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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/value_constraint.hpp>
#include <openscenario_interpreter/utility/compare.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{

ValueConstraint::ValueConstraint(const pugi::xml_node & node, Scope & scope)
: rule(readAttribute<Rule>("rule", node, scope)), value(readAttribute<String>("value", node, scope))
{
}

ValueConstraint::ValueConstraint(const openscenario_msgs::msg::ValueConstraint & msg)
: value(msg.value)
{
  switch (msg.rule) {
    case openscenario_msgs::msg::ValueConstraint::RULE_EQUAL_TO:
      rule.value = Rule::equalTo;
    case openscenario_msgs::msg::ValueConstraint::RULE_GREATER_THAN:
      rule.value = Rule::greaterThan;
    case openscenario_msgs::msg::ValueConstraint::RULE_LESS_THAN:
      rule.value = Rule::lessThan;
    case openscenario_msgs::msg::ValueConstraint::RULE_GREATER_OR_EQUAL:
      rule.value = Rule::greaterOrEqual;
    case openscenario_msgs::msg::ValueConstraint::RULE_LESS_OR_EQUAL:
      rule.value = Rule::lessOrEqual;
    case openscenario_msgs::msg::ValueConstraint::RULE_NOT_EQUAL_TO:
      rule.value = Rule::notEqualTo;
    default:
      throw common::Error(
        "Failed to convert openscenario_msgs::msg::ValueConstraint::rule to "
        "openscenario_interpreter::syntax::Rule");
  }
}

auto ValueConstraint::evaluate(Object & object) -> Object
{
  return asBoolean(compare(object, rule, value));
}

}  // namespace syntax
}  // namespace openscenario_interpreter
