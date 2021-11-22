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
#include <openscenario_interpreter/syntax/act.hpp>
#include <openscenario_interpreter/syntax/maneuver_group.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Act::Act(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope),
  start_trigger(readElement<Trigger>("StartTrigger", node, local()))
{
  callWithElements(node, "ManeuverGroup", 1, unbounded, [&](auto && node) {
    return push_back(readStoryboardElement<ManeuverGroup>(node, local()));
  });

  callWithElements(node, "StopTrigger", 0, 1, [&](auto && node) {
    return stop_trigger.rebind<Trigger>(node, local());
  });
}

auto Act::accomplished() const -> bool
{
  return std::all_of(std::begin(*this), std::end(*this), [&](const Object & each) {
    return each.as<ManeuverGroup>().complete();
  });
}

auto Act::ready() -> bool { return start_trigger.evaluate().as<Boolean>(); }

auto Act::run() -> void
{
  for (auto && each : *this) {
    each.evaluate();
  }
}

auto Act::start() noexcept -> void {}

auto Act::stop() -> void
{
  for (auto && each : *this) {
    each.as<ManeuverGroup>().override();
    each.evaluate();
  }
}

auto Act::stopTriggered() const -> bool
{
  return stop_trigger and stop_trigger.evaluate().as<Boolean>();
}

auto operator<<(nlohmann::json & json, const Act & datum) -> nlohmann::json &
{
  json["name"] = datum.name;

  json["currentState"] = boost::lexical_cast<std::string>(datum.currentState());

  json["ManeuverGroup"] = nlohmann::json::array();

  for (const auto & each : datum) {
    nlohmann::json act;
    act << each.as<ManeuverGroup>();
    json["ManeuverGroup"].push_back(act);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
