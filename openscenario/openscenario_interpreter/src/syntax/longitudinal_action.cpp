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
#include <openscenario_interpreter/syntax/longitudinal_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
LongitudinalAction::LongitudinalAction(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node, {
      {                "SpeedAction", [&](const auto & node) { return make<SpeedAction>(node, scope);                                } },
      { "LongitudinalDistanceAction", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; } },
      {         "SpeedProfileAction", [&](const auto & node) { return make<SpeedProfileAction>(node, scope);                         } },
    }))
// clang-format on
{
}

auto LongitudinalAction::endsImmediately() const -> bool
{
  return apply<bool>([](const auto & action) { return action.endsImmediately(); }, *this);
}

auto LongitudinalAction::run() -> void
{
  return apply<void>([](auto && action) { return action.run(); }, *this);
}

auto LongitudinalAction::start() -> void
{
  return apply<void>([](auto && action) { return action.start(); }, *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
