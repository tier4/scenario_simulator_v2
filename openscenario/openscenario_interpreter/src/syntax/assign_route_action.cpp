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
#include <openscenario_interpreter/syntax/assign_route_action.hpp>
#include <openscenario_interpreter/syntax/catalog_reference.hpp>
#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/route.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
AssignRouteAction::AssignRouteAction(const pugi::xml_node & node, Scope & scope)
// clang-format off
: Scope(scope),
  route(
    choice(node, {
      { "Route",            [&](auto && node) { return make<Route>     (node, local());        } },
      { "CatalogReference", [&](auto && node) { return CatalogReference(node, local()).make(); } },
    }))
// clang-format on
{
  // OpenSCENARIO 1.2 Table 11
  for (const auto & actor : actors) {
    if (auto object_types = actor.objectTypes(); object_types != std::set{ObjectType::vehicle} and
                                                 object_types != std::set{ObjectType::pedestrian}) {
      THROW_SEMANTIC_ERROR(
        "Actors may be either of vehicle type or a pedestrian type;"
        "See OpenSCENARIO 1.2 Table 11 for more details");
    }
  }
}

auto AssignRouteAction::accomplished() noexcept -> bool { return true; }

auto AssignRouteAction::endsImmediately() noexcept -> bool { return true; }

auto AssignRouteAction::run() -> void {}

auto AssignRouteAction::start() -> void
{
  auto get_from_controller_property =
    [this](const EntityRef & entity_ref, const std::string & property_name) -> std::optional<bool> {
    if (const auto & entity = global().entities->ref(entity_ref);
        entity.template is<ScenarioObject>()) {
      const auto & object_controller = entity.as<ScenarioObject>().object_controller;
      if (object_controller.isAutoware() && object_controller.is<Controller>()) {
        auto controller = object_controller.as<Controller>();
        if (controller.properties.contains(property_name)) {
          return controller.properties.template get<Boolean>(property_name);
        }
      }
    }
    return std::nullopt;
  };

  auto get_from_parameter =
    [&](const std::string & parameter_name, const bool default_value) -> bool {
    try {
      return ref<Boolean>(parameter_name);
    } catch (const SyntaxError &) {
      return default_value;
    }
  };

  traffic_simulator::v2::RouteOption route_option;
  route_option.use_lane_ids_for_routing =
    get_from_parameter("RoutingAction__use_lane_ids_for_routing", false);

  for (const auto & actor : actors) {
    actor.apply([&](const auto & object) {
      route_option.allow_goal_modification = [&]() {
        if (auto from_property = get_from_controller_property(object, "allowGoalModification")) {
          // property specification takes precedence
          return from_property.value();
        } else {
          return get_from_parameter("RoutingAction__allow_goal_modification", false);
        }
      }();
      applyAssignRouteAction(
        object, static_cast<std::vector<NativeLanePosition>>(route.as<const Route>()),
        route_option);
    });
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
