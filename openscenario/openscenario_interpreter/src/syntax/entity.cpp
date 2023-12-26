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

namespace openscenario_interpreter
{
inline namespace syntax
{
EntityBase::EntityBase(const ScenarioObject & object) : Object(make(object)) {}

EntityBase::EntityBase(const EntitySelection & object) : Object(make(object)) {}

EntityBase::EntityBase(const String & name, const Scope & scope)
: Object(scope.global().entities->ref(name))
{
}

EntityBase::EntityBase(const pugi::xml_node & node, const Scope & scope)
: EntityBase(readAttribute<String>("entityRef", node, scope), scope)
{
}

auto operator==(const EntityBase & left, const EntityBase & right) -> bool
{
  return left.get() == right.get();
}

auto EntityBase::name() const -> String { return this->as<Scope>().name; }

auto throwIfNotSingle(const Object & entity) -> void
{
  if (not entity.is<ScenarioObject>()) {
    THROW_SEMANTIC_ERROR("Tried to reference `EntitySelection` where it is forbidden.");
  }
}

SingleEntity::SingleEntity(const ScenarioObject & object) : EntityBase(object) {}

SingleEntity::SingleEntity(const String & name, const Scope & scope) : EntityBase(name, scope)
{
  throwIfNotSingle(*this);
}

SingleEntity::SingleEntity(const pugi::xml_node & node, const Scope & scope)
: EntityBase(node, scope)
{
  throwIfNotSingle(*this);
}

SingleEntity::operator String() const { return this->name(); }

SingleEntity::operator EntityRef() const { return this->name(); }

GroupedEntity::GroupedEntity(const SingleEntity & single) : EntityBase(single) {}

auto GroupedEntity::objects() const -> std::set<SingleEntity>
{
  if (is<ScenarioObject>()) {
    return {as<ScenarioObject>()};
  } else if (is<EntitySelection>()) {
    return as<EntitySelection>().objects();
  } else {
    throw std::runtime_error{"Unexpected entity type is detected. This is a simulator bug."};
  }
}

auto GroupedEntity::objectTypes() const -> std::set<ObjectType::value_type>
{
  if (is<ScenarioObject>()) {
    return {as<ScenarioObject>().objectType()};
  } else if (is<EntitySelection>()) {
    return as<EntitySelection>().objectTypes();
  } else {
    throw std::runtime_error{"Unexpected entity type is detected. This is a simulator bug."};
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
