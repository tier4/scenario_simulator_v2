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
#include <iterator>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/by_entity_condition.hpp>
#include <openscenario_interpreter/syntax/by_value_condition.hpp>
#include <openscenario_interpreter/syntax/condition.hpp>
#include <openscenario_interpreter/syntax/condition_edge.hpp>
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
  }

  // push evaluation entry to `evaluation_history`
  // `evaluation_history` contains past evaluation entries in order of evaluation time
  auto latest_time = evaluateSimulationTime();
  auto latest_result = ComplexType::evaluate().as<Boolean>();
  evaluation_history.emplace_back(latest_time, latest_result);

  // find the oldest evaluation entry which was evaluated in the last `delay` time
  auto next_delayed_evaluation = std::find_if(
    std::begin(evaluation_history), std::end(evaluation_history),
    [&](const EvaluationEntry & entry) {
      const auto & [past_time, past_result] = entry;
      return past_time > latest_time - delay;
    });

  // if `next_delayed_evaluation` points the first entry of `evaluation_history`,
  // ongoing simulation did not pass the `delay` time, so simply return `false_v` here
  if (next_delayed_evaluation == std::begin(evaluation_history)) {
    return false_v;
  }
  auto delayed_evaluation = std::prev(next_delayed_evaluation);
  auto [current_time, current_result] = *delayed_evaluation;

  switch (condition_edge) {
    case ConditionEdge::rising:
    case ConditionEdge::falling:
    case ConditionEdge::risingOrFalling: {
      // these condition edges require two evaluation entries,
      // so if `delayed_evaluation` is the only entry, simply return `false_v`
      if (delayed_evaluation == std::begin(evaluation_history)) {
        return false_v;
      }
      auto previous_delayed_evaluation = std::prev(delayed_evaluation);
      auto [previous_time, previous_result] = *previous_delayed_evaluation;

      switch (condition_edge) {
        case ConditionEdge::rising:
          current_value = current_result and not previous_result;
          break;

        case ConditionEdge::falling:
          current_value = not current_result and previous_result;
          break;

        case ConditionEdge::risingOrFalling:
          current_value = current_result != previous_result;
          break;

        default:
          throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(ConditionEdge, condition_edge);
      }

      evaluation_history.erase(std::begin(evaluation_history), previous_delayed_evaluation);
      break;
    }

    case ConditionEdge::none:
    case ConditionEdge::sticky: {
      current_value = current_result;
      evaluation_history.erase(std::begin(evaluation_history), delayed_evaluation);
      break;
    }

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(ConditionEdge, condition_edge);
  }

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
