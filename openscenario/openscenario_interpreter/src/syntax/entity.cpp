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
Entity::Entity(const EntityRef & entity_ref, const Scope & scope)
: Entity(entity_ref, scope.global().entities)
{
}

Entity::Entity(const EntityRef & entity_ref, const Entities * entities)
: entity(entities->ref(entity_ref))
{
}

auto Entity::objects() const -> std::set<EntityRef>
{
  auto objects = std::set<EntityRef>{};
  if (entity.is<ScenarioObject>()) {
    objects.emplace(entity.as<ScenarioObject>().name);
  } else if (entity.is<EntitySelection>()) {
    objects.merge(entity.as<EntitySelection>().objects());
  } else {
    throw std::runtime_error{"Unexpected entity type is detected. This is a simulator bug."};
  }
  return objects;
}

auto Entity::objectTypes() const -> std::set<ObjectType::value_type>
{
  auto types = std::set<ObjectType::value_type>{};
  if (entity.is<ScenarioObject>()) {
    types.emplace(entity.as<ScenarioObject>().objectType());
  } else if (entity.is<EntitySelection>()) {
    types.merge(entity.as<EntitySelection>().objectTypes());
  } else {
    throw std::runtime_error{"Unexpected entity type is detected. This is a simulator bug."};
  }
  return types;
}

Entity::operator String() const
{
  if (entity.is<ScenarioObject>()) {
    throw std::runtime_error{
      "Tried to convert non-ScenarioObject entity to String. This is a simulator bug."};
  }
  return entity.as<ScenarioObject>().name;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
