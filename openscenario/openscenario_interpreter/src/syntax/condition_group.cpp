// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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
#include <openscenario_interpreter/syntax/condition_group.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ConditionGroup::ConditionGroup(const pugi::xml_node & node, Scope & scope) : current_value(false)
{
  callWithElements(
    node, "Condition", 1, unbounded, [&](auto && node) { emplace_back(node, scope); });
}

auto ConditionGroup::evaluate() -> Element
{
  // NOTE: Don't use std::all_of; Intentionally does not short-circuit evaluation.
  return asBoolean(
    current_value = std::accumulate(
      std::begin(*this), std::end(*this), true, [&](auto && lhs, Condition & condition) {
        const auto rhs = condition.evaluate();
        return lhs and rhs.as<Boolean>();
      }));
}

auto operator<<(nlohmann::json & json, const ConditionGroup & datum) -> nlohmann::json &
{
  json["currentValue"] = boost::lexical_cast<std::string>(Boolean(datum.current_value));

  json["Condition"] = nlohmann::json::array();

  for (const auto & each : datum) {
    nlohmann::json condition;
    condition << each;
    json["Condition"].push_back(condition);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
