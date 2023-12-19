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
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/entity_selection.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Entities::Entities(const pugi::xml_node & node, Scope & scope)
{
  scope.global().entities = this;

  traverse<0, unbounded>(node, "ScenarioObject", [&](auto && node) {
    entities.emplace(readAttribute<String>("name", node, scope), make<ScenarioObject>(node, scope));
  });

  traverse<0, unbounded>(node, "EntitySelection", [&](auto && node) {
    entities.emplace(
      readAttribute<String>("name", node, scope), make<EntitySelection>(node, scope));
  });
}

auto Entities::isAdded(const EntityRef & entity_ref) const -> bool
{
  auto object_list = GroupedEntity{entity_ref, *this}.objectNames();
  return std::all_of(std::begin(object_list), std::end(object_list), [&](const String & object) {
    if (auto entity = ref(object); entity.is<ScenarioObject>()) {
      return entity.as<ScenarioObject>().is_added;
    } else {
      THROW_ERROR(
        common::Error, "Unsupported entity type ", std::quoted(makeTypename(entity->type().name())),
        " is detected");
    }
  });
}

auto Entities::ref(const EntityRef & entity_ref) const -> Object
{
  if (auto entry = entities.find(entity_ref); entry == std::end(entities)) {
    throw Error("An undeclared entity ", std::quoted(entity_ref), " was specified in entityRef.");
  } else {
    return entry->second;
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
