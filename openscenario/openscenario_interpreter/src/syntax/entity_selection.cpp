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
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/entity_selection.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/selected_entities.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/utility/overload.hpp>
#include <set>

#include "openscenario_interpreter/syntax/entity.hpp"

namespace openscenario_interpreter
{
inline namespace syntax
{
EntitySelection::EntitySelection(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope),
  SelectedEntities(readElement<SelectedEntities>("Members", node, scope))
{
}

auto EntitySelection::objects() const -> std::set<SingleEntity>
{
  return apply<std::set<SingleEntity>>(
    overload(
      [&](const SelectedEntityRefs & entity_refs) {
        auto result = std::set<SingleEntity>{};
        for (const auto & entity_ref : entity_refs.entityRefs) {
          result.merge(entity_ref.objects());
        }
        return result;
      },
      [&](const SelectedByTypes &) {
        auto result = std::set<SingleEntity>{};
        auto types = objectTypes();
        for (const auto & [name, object] : *global().entities) {
          if (
            object.is<ScenarioObject>() and types.count(object.as<ScenarioObject>().objectType())) {
            result.emplace(name, *this);
          }
        }
        return result;
      }),
    *this);
}

auto EntitySelection::objectTypes() const -> std::set<ObjectType::value_type>
{
  return apply<std::set<ObjectType::value_type>>(
    overload(
      [&](const SelectedEntityRefs & entity_refs) {
        auto result = std::set<ObjectType::value_type>{};
        for (const auto & entity_ref : entity_refs.entityRefs) {
          result.merge(entity_ref.objectTypes());
        }
        return result;
      },
      [&](const SelectedByTypes & by_types) {
        return std::set<ObjectType::value_type>{
          std::begin(by_types.byTypes), std::end(by_types.byTypes)};
      }),
    *this);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
