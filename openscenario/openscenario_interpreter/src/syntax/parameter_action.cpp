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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/parameter_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ParameterAction::ParameterAction(const pugi::xml_node & parent, Scope & scope)
// clang-format off
: ComplexType(
    choice(parent, {
      {    "SetAction", [&](auto && node) { return make<   ParameterSetAction>(node, scope, readAttribute<String>("parameterRef", parent, scope)); } },
      { "ModifyAction", [&](auto && node) { return make<ParameterModifyAction>(node, scope, readAttribute<String>("parameterRef", parent, scope)); } },
    }))
// clang-format on
{
}

auto ParameterAction::endsImmediately() -> bool { return true; }

auto ParameterAction::run() -> void
{
  return apply<void>([](auto && action) { return action.run(); }, *this);
}

auto ParameterAction::start() -> void
{
  return apply<void>([](auto && action) { return action.start(); }, *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
