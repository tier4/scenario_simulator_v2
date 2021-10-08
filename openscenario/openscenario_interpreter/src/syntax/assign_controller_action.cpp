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
#include <openscenario_interpreter/syntax/assign_controller_action.hpp>
#include <openscenario_interpreter/syntax/controller.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
AssignControllerAction::AssignControllerAction(const pugi::xml_node & node, Scope & scope)
// clang-format off
: Scope(scope),
  ComplexType(
    choice(node,
      std::make_pair("Controller",       [&](auto && node) { return make<Controller>(node, localScope()); }),
      std::make_pair("CatalogReference", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; })))
// clang-format on
{
}

auto AssignControllerAction::operator()() const -> void
{
  for (const auto & actor : actors) {
    applyAssignControllerAction(actor, as<Controller>());
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
