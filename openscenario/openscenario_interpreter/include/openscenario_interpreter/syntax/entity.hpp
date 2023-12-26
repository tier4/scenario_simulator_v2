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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_HPP_

#include <algorithm>
#include <cstddef>
#include <functional>
#include <iterator>
#include <openscenario_interpreter/object.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <type_traits>
#include <valarray>
#include <vector>

namespace openscenario_interpreter
{
class Scope;

inline namespace syntax
{
struct Entities;

struct EntityBase : public Object
{
  EntityBase() = default;

  EntityBase(const String & name, const Entities & entities);

  EntityBase(const String & name, Scope & scope);

  EntityBase(const pugi::xml_node & node, Scope & scope);

  auto name() const -> String;
};

auto operator==(const EntityBase & left, const EntityBase & right) -> bool;

struct SingleEntity : public EntityBase
{
  SingleEntity() = default;

  SingleEntity(const String & name, const Entities & entities);

  SingleEntity(const String & name, Scope & scope);

  SingleEntity(const pugi::xml_node & node, Scope & scope);

  operator String() const;

  operator EntityRef() const;
};

struct GroupedEntity : public EntityBase
{
  using EntityBase::EntityBase;

  auto objectNames() const -> std::set<EntityRef>;

  auto objectTypes() const -> std::set<ObjectType::value_type>;

  template <typename Function>
  auto apply(const Function & function) const
  {
    using Result = std::invoke_result_t<Function, String>;
    auto objects = this->objectNames();
    if constexpr (std::is_same_v<Result, void>) {
      std::for_each(std::begin(objects), std::end(objects), function);
    } else {
      auto results = std::valarray<Result>(objects.size());
      std::transform(std::begin(objects), std::end(objects), std::begin(results), function);
      return results;
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

template <>
struct std::hash<openscenario_interpreter::SingleEntity>
{
  auto operator()(const openscenario_interpreter::SingleEntity & entity) const -> std::size_t
  {
    return std::hash<void *>{}(entity.get());
  }
};

template <>
struct std::hash<openscenario_interpreter::GroupedEntity>
{
  auto operator()(const openscenario_interpreter::GroupedEntity & entity) const -> std::size_t
  {
    return std::hash<void *>{}(entity.get());
  }
};

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_HPP_
