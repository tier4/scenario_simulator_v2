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

auto SelectedEntityRefs::objects(const Entities & entities) -> std::list<String>
{
  return entities.objects(std::list<String>(std::begin(entityRefs), std::end(entityRefs)));
}

SelectedByTypes::SelectedByTypes(const pugi::xml_node & tree, Scope & scope)
: byTypes(readElements<ByType, 0>("ByType", tree, scope))
{
}

auto SelectedByTypes::objects(const Entities & entities) -> std::list<String>
{
  auto selected_entities = std::list<String>();
  for (const auto & [name, object] : entities) {
    for (const auto & byType : byTypes) {
      switch (byType.objectType) {
      case ObjectType::vehicle:
        if (object.is<Vehicle>()) {
          selected_entities.emplace_back(name);
        }
        break;
      case ObjectType::pedestrian:
        if (object.is<Pedestrian>()) {
          selected_entities.emplace_back(name);
        }
        break;
      case ObjectType::miscellaneous:
        if (object.is<MiscObject>()) {
          selected_entities.emplace_back(name);
        }
        break;
      case ObjectType::external:
        THROW_SYNTAX_ERROR("Selecting external object is not supported yet");
        break;
      default:
        THROW_SYNTAX_ERROR(
          "Unexpected entity ", std::quoted(name), " of type ",
          std::quoted(makeTypename(object->type().name())), " is detected; this is a bug");
        break;
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

auto SelectedEntities::objects(const Entities & entities) -> std::list<String>
{
  return apply<std::list<String>>([&](auto & e) { return e.objects(entities); }, *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
