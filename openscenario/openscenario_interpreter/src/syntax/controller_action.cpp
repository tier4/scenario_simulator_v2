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
#include <openscenario_interpreter/syntax/controller_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ControllerAction::ControllerAction(const pugi::xml_node & node, Scope & scope)
: Scope(scope),
  assign_controller_action(
    readElement<AssignControllerAction>("AssignControllerAction", node, scope)),
  override_controller_value_action(readElement<OverrideControllerValueAction>(
    "OverrideControllerValueAction", node, scope))  // NOTE: DUMMY IMPLEMENTATION
{
}

auto ControllerAction::accomplished() noexcept -> bool  //
{
  return true;
}

auto ControllerAction::endsImmediately() noexcept -> bool  //
{
  return true;
}

auto ControllerAction::run() noexcept -> void {}

auto ControllerAction::start() const -> void
{
  for (const auto & actor : actors) {
    assign_controller_action(actor);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
