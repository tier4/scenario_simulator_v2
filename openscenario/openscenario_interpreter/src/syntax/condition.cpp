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

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <numeric>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/by_entity_condition.hpp>
#include <openscenario_interpreter/syntax/by_value_condition.hpp>
#include <openscenario_interpreter/syntax/condition.hpp>
#include <openscenario_interpreter/syntax/condition_edge.hpp>
#include <openscenario_interpreter/utility/demangle.hpp>

#include "openscenario_interpreter/object.hpp"

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
  using ResultSet = std::bitset<8>;

  auto update_condition = [&](std::size_t number, const auto & evaluator) -> bool {
    auto latest_time = evaluateSimulationTime();
    auto latest_result = ComplexType::evaluate().as<Boolean>();
    evaluation_history.emplace_back(latest_time, latest_result);

    auto canary_iterator = std::find_if(
      std::begin(evaluation_history), std::end(evaluation_history), [&](EvaluationEntry entry) {
        auto [time, result] = entry;
        return time > latest_time - delay;
      });

    auto results = ResultSet();
    auto entry_count = std::size_t(0);
    auto entry_iterator = std::reverse_iterator(canary_iterator);
    while (entry_count < number and entry_iterator != std::rend(evaluation_history)) {
      auto [time, result] = *entry_iterator;
      results[entry_count] = result;
      ++entry_iterator;
      ++entry_count;
    }

    if (entry_count == number) {
      evaluation_history.erase(std::begin(evaluation_history), entry_iterator.base());
      return evaluator(results);
    } else {
      return false;
    }
  };

  current_value = [&] {
    switch (condition_edge) {
      case ConditionEdge::rising:
        return update_condition(
          2, [](ResultSet results) { return results.test(0) and not results.test(1); });

      case ConditionEdge::falling:
        return update_condition(
          2, [](ResultSet results) { return not results.test(0) and results.test(1); });

      case ConditionEdge::risingOrFalling:
        return update_condition(
          2, [](ResultSet results) { return results.test(0) xor results.test(1); });

      case ConditionEdge::none:
        return update_condition(1, [](ResultSet results) { return results.test(0); });

      case ConditionEdge::sticky:
        return current_value ||
               update_condition(1, [](ResultSet results) { return results.test(0); });

      default:
        throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(ConditionEdge, condition_edge);
    }
  }();
  return asBoolean(current_value);
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
