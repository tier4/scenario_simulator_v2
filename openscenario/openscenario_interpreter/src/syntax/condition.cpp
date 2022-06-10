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
#include <openscenario_interpreter/syntax/by_entity_condition.hpp>
#include <openscenario_interpreter/syntax/by_value_condition.hpp>
#include <openscenario_interpreter/syntax/condition.hpp>
#include <openscenario_interpreter/utility/demangle.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<ConditionEdge>::value, "");

static_assert(std::is_trivial<ConditionEdge>::value, "");

Condition::Condition(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node,
      std::make_pair("ByEntityCondition", [&](auto && node) { return make<ByEntityCondition>(node, scope); }),
      std::make_pair( "ByValueCondition", [&](auto && node) { return make< ByValueCondition>(node, scope); }))),
  name(readAttribute<String>("name", node, scope)),
  delay(readAttribute<Double>("delay", node, scope, Double())),
  condition_edge(readAttribute<ConditionEdge>("conditionEdge", node, scope)),
  current_value(false)
// clang-format on
{
}

auto Condition::evaluate() -> Object
{
  if (condition_edge == ConditionEdge::sticky and current_value) {
    return true_v;
  } else {
    return asBoolean(current_value = Object::evaluate().as<Boolean>());
  }
}

auto operator<<(nlohmann::json & json, const Condition & datum) -> nlohmann::json &
{
  json["currentEvaluation"] = datum.description();

  json["currentValue"] = boost::lexical_cast<std::string>(Boolean(datum.current_value));

  json["name"] = datum.name;

  // clang-format off
  static const std::unordered_map<
    std::type_index,
    std::function<std::string(const Condition &)>> table
  {
    { typeid(ByEntityCondition), [&](const Condition & condition) { return makeTypename(condition.as<ByEntityCondition>().type()); } },
    { typeid( ByValueCondition), [&](const Condition & condition) { return makeTypename(condition.as< ByValueCondition>().type()); } },
  };
  // clang-format on

  json["type"] = table.at(datum.type())(datum);

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
