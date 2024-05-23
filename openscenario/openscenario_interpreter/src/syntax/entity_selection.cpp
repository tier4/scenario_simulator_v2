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
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/entity_selection.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/selected_entities.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/utility/overload.hpp>
#include <set>

namespace openscenario_interpreter
{
inline namespace syntax
{
EntitySelection::EntitySelection(const pugi::xml_node & node, Scope & scope)
: Scope(readAttribute<String>("name", node, scope), scope),
  SelectedEntities(readElement<SelectedEntities>("Members", node, scope))
{
}

auto EntitySelection::objects() const -> std::set<Entity>
{
  auto result = std::set<Entity>{};
  if (not entityRef.empty()) {
    for (const auto & entity_ref : entityRef) {
      result.merge(entity_ref.objects());
    }
  } else {
    auto types = objectTypes();
    for (const auto & [name, object] : *global().entities) {
      if (object.is<ScenarioObject>() and types.count(object.as<ScenarioObject>().objectType())) {
        result.emplace(name, *this);
      }
    }
  }
  return result;
}

auto EntitySelection::objectTypes() const -> std::set<ObjectType::value_type>
{
  auto result = std::set<ObjectType::value_type>{};
  if (not entityRef.empty()) {
    for (const auto & entity_ref : entityRef) {
      result.merge(entity_ref.objectTypes());
    }
  } else {
    result.insert(byType.cbegin(), byType.cend());
  }
  return result;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
