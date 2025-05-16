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
#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
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
    [](
      const WorldPosition & position, auto && actor,
      const traffic_simulator::RouteOptions & options) {
      return applyAcquirePositionAction(actor, static_cast<NativeWorldPosition>(position), options);
    },
    [](
      const RelativeWorldPosition & position, auto && actor,
      const traffic_simulator::RouteOptions & options) {
      return applyAcquirePositionAction(actor, static_cast<NativeLanePosition>(position), options);
    },
    [](
      const RelativeObjectPosition & position, auto && actor,
      const traffic_simulator::RouteOptions & options) {
      return applyAcquirePositionAction(actor, static_cast<NativeLanePosition>(position), options);
    },
    [](
      const LanePosition & position, auto && actor,
      const traffic_simulator::RouteOptions & options) {
      return applyAcquirePositionAction(actor, static_cast<NativeLanePosition>(position), options);
    });

  auto allow_goal_modification_from_parameter = [&]() -> bool {
    try {
      return ref<Boolean>(std::string("AcquirePositionAction.allow_goal_modification"));
    } catch (const SyntaxError &) {
      // default value of allow_goal_modification
      return false;
    }
  }();

  auto get_allow_goal_modification_from_property =
    [this](const EntityRef & entity_ref) -> std::optional<bool> {
    if (const auto & entity = global().entities->ref(entity_ref);
        entity.template is<ScenarioObject>()) {
      const auto & object_controller = entity.as<ScenarioObject>().object_controller;
      if (object_controller.isAutoware() && object_controller.is<Controller>()) {
        auto controller = object_controller.as<Controller>();
        if (controller.properties.contains("allowGoalModification")) {
          return controller.properties.template get<Boolean>("allowGoalModification");
        }
      }
    }
    return std::nullopt;
  };

  for (const auto & actor : actors) {
    actor.apply([&](const auto & object) {
      traffic_simulator::RouteOptions options;
      options.allow_goal_modification = [&]() {
        if (auto allow_goal_modification_from_property =
              get_allow_goal_modification_from_property(object);
            allow_goal_modification_from_property.has_value()) {
          // property specification takes precedence
          return allow_goal_modification_from_property.value();
        } else {
          return allow_goal_modification_from_parameter;
        }
      }();

      apply<void>(acquire_position, position, object, options);
    });
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
