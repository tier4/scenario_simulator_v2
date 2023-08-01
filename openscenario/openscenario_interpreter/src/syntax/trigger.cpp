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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/trigger.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Trigger::Trigger(const pugi::xml_node & node, Scope & scope)
{
  traverse<0, unbounded>(node, "ConditionGroup", [&](auto && node) { emplace_back(node, scope); });
}

auto Trigger::evaluate() -> Object
{
  /* -------------------------------------------------------------------------
   *
   *  A trigger is then defined as an association of condition groups. A
   *  trigger evaluates to true if at least one of the associated condition
   *  groups evaluates to true, otherwise it evaluates to false (OR
   *  operation).
   *
   * ---------------------------------------------------------------------- */
  // NOTE: Don't use std::any_of; Intentionally does not short-circuit evaluation.
  try {
    return asBoolean(
      current_value = std::accumulate(
        std::begin(*this), std::end(*this), false,
        [&](auto && lhs, ConditionGroup & condition_group) {
          const auto rhs = condition_group.evaluate();
          return lhs or rhs.as<Boolean>();
        }));
  } catch (...) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("#######"), "throw Trigger evaluate");
    throw;
  }
}

auto Trigger::activeConditionGroupIndex() const -> int
{
  size_t index{0};
  for (auto it = begin(); it != end(); ++it, ++index) {
    ConditionGroup const & cgroup = *it;
    bool all_conditions_true = std::all_of(
      cgroup.begin(), cgroup.end(), [](const Condition & c) { return c.current_value; });
    if (all_conditions_true) break;
  }
  return index;
}

auto Trigger::activeConditionGroupDescription() const
  -> std::vector<std::pair<std::string, std::string>>
{
  auto it = begin();
  std::advance(it, activeConditionGroupIndex());
  ConditionGroup const & cgroup = *it;
  RCLCPP_WARN_STREAM(
    rclcpp::get_logger("XXXXX"),
    "size: " << cgroup.size() << " index: " << activeConditionGroupIndex());
  std::vector<std::pair<std::string, std::string>> list;
  for (auto itt = cgroup.begin(); itt != cgroup.end(); ++itt)
    list.push_back(std::make_pair(itt->name, itt->description()));
  return list;
}

auto operator<<(nlohmann::json & json, const Trigger & datum) -> nlohmann::json &
{
  json["currentValue"] = boost::lexical_cast<std::string>(Boolean(datum.current_value));

  json["ConditionGroup"] = nlohmann::json::array();

  for (const auto & each : datum) {
    nlohmann::json condition_group;
    condition_group << each;
    json["ConditionGroup"].push_back(condition_group);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
