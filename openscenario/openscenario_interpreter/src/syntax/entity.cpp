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

#include <iterator>
#include <openscenario_interpreter/object.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/entity_selection.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/selected_entities.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/utility/overload.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
Entity::Entity(EntityRef entity_ref, const Scope & scope)
: Entity(std::move(entity_ref), scope.global().entities)
{
}

Entity::Entity(EntityRef entity_ref, const Entities * entities)
: entity_ref(std::move(entity_ref)), entities(entities)
{
}

auto Entity::objects() const -> std::set<EntityRef>
{
  auto objects = std::set<EntityRef>{};
  auto entity = entities->ref(entity_ref);
  if (entity.is<ScenarioObject>()) {
    objects.emplace(entity_ref);
  } else if (entity.is<EntitySelection>()) {
    auto selection = entity.as<EntitySelection>();
    if (selection.is<SelectedEntityRefs>()) {
      for (const auto & entity : selection.as<SelectedEntityRefs>().entityRefs) {
        objects.merge(entity.objects());
      }
    } else if (selection.is<SelectedByTypes>()) {
      const auto & by_types = selection.as<SelectedByTypes>().byTypes;
      auto types = std::set<ObjectType::value_type>{std::begin(by_types), std::end(by_types)};
      for (const auto & [name, object] : entities->entities) {
        if (
          object.is<ScenarioObject>() and
          types.count(ObjectType::of(object.as<ScenarioObject>()))) {
          objects.emplace(name);
        }
      }
    }
  } else {
    throw std::runtime_error{"Unexpected entity type is detected. This is a simulator bug."};
  }
  return objects;
}

auto Entity::types() const -> std::set<ObjectType::value_type>
{
  auto types = std::set<ObjectType::value_type>{};
  auto entity = entities->ref(entity_ref);
  if (entity.is<ScenarioObject>()) {
    types.emplace(ObjectType::of(entity.as<ScenarioObject>()));
  } else if (entity.is<EntitySelection>()) {
    auto selection = entity.as<EntitySelection>();
    if (selection.is<SelectedEntityRefs>()) {
      for (const auto & entity : selection.as<SelectedEntityRefs>().entityRefs) {
        types.merge(entity.types());
      }
    } else if (selection.is<SelectedByTypes>()) {
      const auto & by_types = selection.as<SelectedByTypes>().byTypes;
      types.insert(std::begin(by_types), std::end(by_types));
    }
  } else {
    throw std::runtime_error{"Unexpected entity type is detected. This is a simulator bug."};
  }
  return types;
}

Entity::operator String() const
{
  if (not entities->ref(entity_ref).is<ScenarioObject>()) {
    throw std::runtime_error{
      "Tried to convert non-ScenarioObject entity to String. This is a simulator bug."};
  }
  return entity_ref;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
