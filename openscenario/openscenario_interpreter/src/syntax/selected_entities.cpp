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

auto SelectedEntityRefs::enumerate(const Entities & entities) -> std::list<String>
{
  auto selected_entities = std::list<String>();
  for (const auto & entityRef : entityRefs) {
    auto object = entities.ref(entityRef);
    if (object.is<ScenarioObject>()) {
      selected_entities.emplace_back(entityRef);
    } else if (object.is<EntitySelection>()) {
      selected_entities.merge(object.as<EntitySelection>().enumerate(entities));
    } else {
      THROW_SYNTAX_ERROR(
        "Unexpected entity ", std::quoted(entityRef), " of type ",
        std::quoted(object->type().name()), " is detected; this is a bug");
    }
  }
  return selected_entities;
}

SelectedByTypes::SelectedByTypes(const pugi::xml_node & tree, Scope & scope)
: byTypes(readElements<ByType, 0>("ByType", tree, scope))
{
}

auto SelectedByTypes::enumerate(const Entities & entities) -> std::list<String>
{
  auto selected_entities = std::list<String>();
  for (const auto & [name, object] : entities) {
    for (const auto & byType : byTypes) {
      if (
        (object.is<Vehicle>() and byType.objectType == ObjectType::vehicle) or
        (object.is<Pedestrian>() and byType.objectType == ObjectType::pedestrian) or
        (object.is<MiscObject>() and byType.objectType == ObjectType::miscellaneous)) {
        selected_entities.emplace_back(name);
      } else if (byType.objectType == ObjectType::external) {
        THROW_SYNTAX_ERROR("Selecting external object is not supported yet");
      } else {
        THROW_SYNTAX_ERROR(
          "Unexpected entity ", std::quoted(name), " of type ",
          std::quoted(object->type().name()), " is detected; this is a bug");
      }
    }
  }
  return selected_entities;
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

auto SelectedEntities::enumerate(const Entities & entities) -> std::list<String>
{
  return apply<std::list<String>>([&](auto & e) { return e.enumerate(entities); }, *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
