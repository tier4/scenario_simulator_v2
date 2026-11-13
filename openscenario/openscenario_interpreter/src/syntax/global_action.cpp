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
#include <openscenario_interpreter/syntax/global_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
GlobalAction::GlobalAction(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node, {
      {   "EnvironmentAction", [&](auto && node) { return make<   EnvironmentAction>(node, scope);                       } },
      {        "EntityAction", [&](auto && node) { return make<        EntityAction>(node, scope);                       } },
      {     "ParameterAction", [&](auto && node) { return make<     ParameterAction>(node, scope);                       } },
      {"InfrastructureAction", [&](auto && node) { return make<InfrastructureAction>(node, scope);                       } },
      {       "TrafficAction", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; } },
    }))
// clang-format on
{
}

auto GlobalAction::endsImmediately() const -> bool
{
  return apply<bool>([](const auto & action) { return action.endsImmediately(); }, *this);
}

auto GlobalAction::evaluate() -> Object
{
  assert(endsImmediately());  // NOTE: Called from `InitActions::evaluate`
  apply<void>([](auto && action) { return action.start(); }, *this);
  apply<void>([](auto && action) { return action.run(); }, *this);
  return unspecified;
}

auto GlobalAction::run() -> void
{
  return apply<void>([](auto && action) { return action.run(); }, *this);
}

auto GlobalAction::start() -> void
{
  return apply<void>([](auto && action) { return action.start(); }, *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
