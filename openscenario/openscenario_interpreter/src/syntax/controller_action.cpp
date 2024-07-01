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
#include <openscenario_interpreter/syntax/controller_action.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>

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
  // OpenSCENARIO 1.2 Table 11
  for (const auto & actor : actors) {
    for (const auto & object_type : actor.objectTypes()) {
      if (object_type != ObjectType::vehicle and object_type != ObjectType::pedestrian) {
        THROW_SEMANTIC_ERROR(
          "Actors may be either of vehicle type or a pedestrian type;"
          "See OpenSCENARIO 1.2 Table 11 for more details");
      }
    }
  }
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
    actor.apply(assign_controller_action);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
