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
#include <openscenario_interpreter/syntax/entity_action.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
EntityAction::EntityAction(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node,
      std::make_pair(   "AddEntityAction", [&](auto && node) { return make<   AddEntityAction>(node, scope); }),
      std::make_pair("DeleteEntityAction", [&](auto && node) { return make<DeleteEntityAction>(node, scope); }))),
  entity_ref(readAttribute<String>("entityRef", node, scope), scope)
// clang-format on
{
}

auto EntityAction::accomplished() noexcept -> bool { return endsImmediately(); }

auto EntityAction::endsImmediately() noexcept -> bool { return true; }

auto EntityAction::run() noexcept -> void {}

auto EntityAction::start() const -> void
{
  apply<void>([&](auto && action) { entity_ref.apply(action); }, *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
