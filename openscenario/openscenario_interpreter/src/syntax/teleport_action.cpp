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

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/add_entity_action.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/teleport_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto TeleportAction::accomplished() noexcept -> bool { return true; }

auto TeleportAction::endsImmediately() noexcept -> bool { return true; }

auto TeleportAction::evaluate() const -> Element
{
  run();
  return unspecified;
}

auto TeleportAction::run() const -> void
{
  auto teleport_action = overload(
    [](const WorldPosition & position, const auto & actor) {
      return applyTeleportAction(actor, static_cast<geometry_msgs::msg::Pose>(position));
    },
    [](const RelativeWorldPosition & position, const auto & actor) {
      return applyTeleportAction(
        actor,
        position.reference,  // name
        position,            // geometry_msgs::msg::Point
        position.orientation);
    },
    [](const LanePosition & position, const auto & actor) {
      return applyTeleportAction(actor, static_cast<openscenario_msgs::msg::LaneletPose>(position));
    });

  for (const auto & actor : actors) {
    if (not global().entities.at(actor).as<ScenarioObject>().is_added) {
      AddEntityAction(localScope(), position)(actor);  // NOTE: Tier IV extension
    } else {
      apply<void>(teleport_action, position, actor);
    }
  }
}

auto TeleportAction::start() noexcept -> void {}
}  // namespace syntax
}  // namespace openscenario_interpreter
