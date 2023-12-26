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
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/route.hpp>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
AssignRouteAction::AssignRouteAction(const pugi::xml_node & node, Scope & scope)
// clang-format off
: Scope(scope),
  route(
    choice(node,
      std::make_pair("Route",            [&](auto && node) { return make<Route>     (node, local());        }),
      std::make_pair("CatalogReference", [&](auto && node) { return CatalogReference(node, local()).make(); })))
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
  for (const auto & actor : actors) {
    actor.apply([&](const auto & object) {
      applyAssignRouteAction(
        object, static_cast<std::vector<NativeLanePosition>>(route.as<const Route>()));
    });
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
