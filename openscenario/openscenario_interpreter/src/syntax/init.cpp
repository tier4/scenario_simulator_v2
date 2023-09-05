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
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
#include <openscenario_interpreter/syntax/init.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Init::Init(const pugi::xml_node & node, Scope & scope)
: actions(readElement<InitActions>("Actions", node, scope))
{
}

auto Init::endsImmediately() const -> bool { return actions.endsImmediately(); }

auto Init::evaluateInstantaneousActions() -> Object
{
  try {
    actions.startInstantaneousActions();
    actions.runInstantaneousActions();
    return unspecified;
  } catch (const SpecialAction<EXIT_FAILURE> & action) {
    throw SpecialAction<EXIT_FAILURE>("OpenSCENARIO.Storyboard.Init", action);
  }
}

auto Init::runNonInstantaneousActions() -> void { actions.runNonInstantaneousActions(); }

auto Init::startNonInstantaneousActions() -> void { actions.startNonInstantaneousActions(); }

auto operator<<(nlohmann::json & json, const Init & datum) -> nlohmann::json &
{
  json["Actions"] << datum.actions;

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
