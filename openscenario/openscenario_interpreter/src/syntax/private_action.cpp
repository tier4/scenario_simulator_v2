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
#include <openscenario_interpreter/syntax/private_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
PrivateAction::PrivateAction(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node, {
      {       "LongitudinalAction", [&](const auto & node) { return make<LongitudinalAction>(node, scope);                         } },
      {            "LateralAction", [&](const auto & node) { return make<     LateralAction>(node, scope);                         } },
      {         "VisibilityAction", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; } },
      {        "SynchronizeAction", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; } },
      { "ActivateControllerAction", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; } },
      {         "ControllerAction", [&](const auto & node) { return make<  ControllerAction>(node, scope);                         } },
      {           "TeleportAction", [&](const auto & node) { return make<    TeleportAction>(node, scope);                         } },
      {            "RoutingAction", [&](const auto & node) { return make<     RoutingAction>(node, scope);                         } },
    }))
// clang-format on
{
}

auto PrivateAction::endsImmediately() const -> bool
{
  return apply<bool>([](const auto & action) { return action.endsImmediately(); }, *this);
}

auto PrivateAction::run() -> void
{
  return apply<void>([](auto && action) { return action.run(); }, *this);
}

auto PrivateAction::start() -> void
{
  return apply<void>([](auto && action) { return action.start(); }, *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
