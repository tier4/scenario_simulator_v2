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
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/add_entity_action.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/teleport_action.hpp>
#include <openscenario_interpreter/utility/overload.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
AddEntityAction::AddEntityAction(const pugi::xml_node & node, Scope & scope)
: Scope(scope), position(readElement<Position>("Position", node, scope))
{
}

AddEntityAction::AddEntityAction(const Scope & scope, const Position & position)
: Scope(scope), position(position)
{
}

auto AddEntityAction::accomplished() noexcept -> bool  //
{
  return endsImmediately();
}

auto AddEntityAction::endsImmediately() noexcept -> bool  //
{
  return true;
}

auto AddEntityAction::operator()(const EntityRef & entity_ref) const -> void
try {
  const auto entity = global().entities.at(entity_ref);

  const auto add_entity = overload(
    [&](const Vehicle & vehicle) {
      if (applyAddEntityAction(
            entity_ref,                                                            //
            static_cast<traffic_simulator_msgs::msg::VehicleParameters>(vehicle),  //
            entity.as<ScenarioObject>().object_controller.isUserDefinedController()
              ? traffic_simulator::VehicleBehavior::autoware()
              : traffic_simulator::VehicleBehavior::defaultBehavior())) {
        TeleportAction::teleport(entity_ref, position);
        AssignControllerAction(entity.as<ScenarioObject>().object_controller)(entity_ref);
        entity.as<ScenarioObject>().activateSensors();
        entity.as<ScenarioObject>().activateOutOfRangeMetric(vehicle);
      }
    },
    [&](const Pedestrian & pedestrian) {
      applyAddEntityAction(
        entity_ref, static_cast<traffic_simulator_msgs::msg::PedestrianParameters>(pedestrian));
      TeleportAction::teleport(entity_ref, position);
    },
    [&](const MiscObject & misc_object) {
      applyAddEntityAction(
        entity_ref, static_cast<traffic_simulator_msgs::msg::MiscObjectParameters>(misc_object));
      TeleportAction::teleport(entity_ref, position);
    });

  if (not std::exchange(entity.as<ScenarioObject>().is_added, true)) {
    apply<void>(add_entity, entity.as<EntityObject>());
  } else {
    throw SemanticError(
      "Applying action AddEntityAction to an entity ", std::quoted(entity_ref),
      " that has already been added.");
  }
} catch (const std::out_of_range &) {
  throw SemanticError("No such name of entity ", std::quoted(entity_ref));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
