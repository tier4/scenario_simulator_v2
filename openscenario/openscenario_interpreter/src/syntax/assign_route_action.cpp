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
#include <openscenario_interpreter/syntax/assign_route_action.hpp>
#include <openscenario_interpreter/syntax/catalog_reference.hpp>
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
      std::make_pair("Route",            [&](auto && node) { return                   make<Route>(node, local()); }),
      std::make_pair("CatalogReference", [&](auto && node) { return CatalogReference::make<Route>(node, local()); })))
// clang-format on
{
}

auto AssignRouteAction::accomplished() noexcept -> bool { return true; }

auto AssignRouteAction::endsImmediately() noexcept -> bool { return true; }

auto AssignRouteAction::run() -> void
{
  for (const auto & actor : actors) {
    applyAssignRouteAction(
      actor,
      static_cast<std::vector<traffic_simulator_msgs::msg::LaneletPose>>(route.as<const Route>()));
  }
}

auto AssignRouteAction::start() -> void {}
}  // namespace syntax
}  // namespace openscenario_interpreter
