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
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/entity_selection.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/syntax/time_to_collision_condition_target.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TimeToCollisionConditionTarget::TimeToCollisionConditionTarget(
  const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(choice(node,
    std::make_pair( "Position", [&](auto && node) { return make<Position>(std::forward<decltype(node)>(node), scope); }),
    std::make_pair("EntityRef", [&](auto && node) { return make<  Entity>(std::forward<decltype(node)>(node), scope); })))
// clang-format on
{
  if (is<EntitySelection>()) {
    throw SyntaxError("EntitySelection is not allowed in TimeToCollisionConditionTarget.entityRef");
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
