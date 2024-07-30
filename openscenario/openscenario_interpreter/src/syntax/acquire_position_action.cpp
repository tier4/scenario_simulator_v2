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
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/acquire_position_action.hpp>
#include <openscenario_interpreter/utility/overload.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
AcquirePositionAction::AcquirePositionAction(const pugi::xml_node & node, Scope & scope)
: Scope(scope), position(readElement<Position>("Position", node, local()))
{
}

auto AcquirePositionAction::start() -> void
{
  const auto acquire_position = overload(
    [](const WorldPosition & position, auto && actor) {
      return applyAcquirePositionAction(actor, static_cast<NativeWorldPosition>(position));
    },
    [](const RelativeWorldPosition & position, auto && actor) {
      return applyAcquirePositionAction(actor, static_cast<NativeLanePosition>(position));
    },
    [](const RelativeObjectPosition & position, auto && actor) {
      return applyAcquirePositionAction(actor, static_cast<NativeLanePosition>(position));
    },
    [](const LanePosition & position, auto && actor) {
      return applyAcquirePositionAction(actor, static_cast<NativeLanePosition>(position));
    });

  for (const auto & actor : actors) {
    actor.apply([&](const auto & object) { apply<void>(acquire_position, position, object); });
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
