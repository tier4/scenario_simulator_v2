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
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/entity_selection.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/selected_entities.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
SelectedEntityRefs::SelectedEntityRefs(const pugi::xml_node & tree, Scope & scope)
: entityRefs(readElements<EntityRef, 0>("EntityRef", tree, scope, scope.entities(true)))
{
}

auto SelectedEntityRefs::objects(const Entities & entities) -> std::set<EntityRef>
{
  return entities.objects(entityRefs);
}

auto SelectedEntityRefs::objectTypes(const Entities & entities) -> std::set<ObjectType::value_type>
{
  return entities.objectTypes(entityRefs);
}

SelectedByTypes::SelectedByTypes(const pugi::xml_node & tree, Scope & scope)
: byTypes(readElements<ByType, 0>("ByType", tree, scope))
{
}

auto SelectedByTypes::objects(const Entities & entities) -> std::set<EntityRef>
{
  auto selected_entities = std::set<EntityRef>();
  auto object_types = objectTypes(entities);
  for (const auto & [name, object] : entities.entities) {
    if (
      object.is<ScenarioObject>() and
      object_types.count(object.as<ScenarioObject>().objectType())) {
      selected_entities.emplace(name);
    }
  }
  return selected_entities;
}

auto SelectedByTypes::objectTypes(const Entities &) -> std::set<ObjectType::value_type>
{
  return std::set<ObjectType::value_type>(std::begin(byTypes), std::end(byTypes));
}

SelectedEntities::SelectedEntities(const pugi::xml_node & tree, Scope & scope)
// clang-format off
: ComplexType(
    choice(tree,
      std::make_pair("EntityRef", [&](const auto & tree) { return make<SelectedEntityRefs>(tree.parent(), scope); }),
      std::make_pair(   "ByType", [&](const auto & tree) { return make<   SelectedByTypes>(tree.parent(), scope); })))
// clang-format on
{
}

auto SelectedEntities::objects(const Entities & entities) -> std::set<EntityRef>
{
  return apply<std::set<EntityRef>>([&](auto & e) { return e.objects(entities); }, *this);
}

auto SelectedEntities::objectTypes(const Entities & entities) -> std::set<ObjectType::value_type>
{
  return apply<std::set<ObjectType::value_type>>(
    [&](auto & e) { return e.objectTypes(entities); }, *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
