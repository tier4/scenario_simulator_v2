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

#include <openscenario_interpreter/object.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/by_type.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/entity_selection.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/selected_entities.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
SelectedEntities::SelectedEntities(const pugi::xml_node & tree, Scope & scope)
: entityRef(readElements<Entity, 0>("EntityRef", tree, scope)),
  byType(readElements<ByType, 0>("ByType", tree, scope))
{
  // clang-format off
  // This function call is added to check the correctness of the syntax.
  // DO NOT REMOVE unless syntax check is conducted in another way.
  choice(tree,
    std::pair{"EntityRef", [](const auto &) { return unspecified; }},
    std::pair{"ByType",    [](const auto &) { return unspecified; }});
  // clang-format on
}
}  // namespace syntax
}  // namespace openscenario_interpreter
