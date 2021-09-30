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

#include <openscenario_interpreter/syntax/maneuver_group.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto ManeuverGroup::accomplished() const -> bool
{
  // A ManeuverGroup's goal is accomplished when all its Maneuvers are in the completeState.
  return std::all_of(std::begin(*this), std::end(*this), [&](auto && each) {
    return each.template as<Maneuver>().complete();
  });
}

auto ManeuverGroup::ready() noexcept -> bool { return true; }

auto ManeuverGroup::run() -> void
{
  for (auto && each : *this) {
    each.evaluate();
  }
}

auto ManeuverGroup::start() -> void
{
  for (auto && each : *this) {
    each.as<Maneuver>().changeStateIf(true, standby_state);
  }
}

auto ManeuverGroup::stop() -> void
{
  for (auto && each : *this) {
    each.as<Maneuver>().override();
    each.evaluate();
  }
}

auto ManeuverGroup::stopTriggered() noexcept -> bool { return false; }

auto operator<<(nlohmann::json & json, const ManeuverGroup & datum) -> nlohmann::json &
{
  json["name"] = datum.name;

  json["currentState"] = boost::lexical_cast<std::string>(datum.currentState());

  json["currentExecutionCount"] = datum.current_execution_count;
  json["maximumExecutionCount"] = datum.maximum_execution_count;

  json["Maneuver"] = nlohmann::json::array();

  for (const auto & each : datum) {
    nlohmann::json maneuver;
    maneuver << each.as<Maneuver>();
    json["Maneuver"].push_back(maneuver);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
