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
struct Scope;

inline namespace syntax
{
struct Entities;

struct ScenarioObject;

struct EntitySelection;

struct Entity : public Object
{
  Entity() = default;

  Entity(const ScenarioObject &);

  Entity(const EntitySelection &);

  Entity(const String &, const Scope &);

  Entity(const pugi::xml_node &, const Scope &);

  auto name() const -> String;

  auto objects() const -> std::set<Entity>;

  auto objectTypes() const -> std::set<ObjectType::value_type>;

  template <typename Function>
  auto apply(const Function & function) const
  {
    using Result = std::invoke_result_t<Function, Entity>;
    auto objects = this->objects();
    if constexpr (std::is_same_v<Result, void>) {
      std::for_each(std::begin(objects), std::end(objects), function);
    } else {
      auto results = std::valarray<Result>(objects.size());
      std::transform(std::begin(objects), std::end(objects), std::begin(results), function);
      return results;
    }
  }

  /**
   * This function is for ScenarioObject only.
   * @throws std::runtime_error if the entity is not a ScenarioObject.
   * @note To iterate over all objects, use `apply` function instead.
   * @note To get the name of the entity, use `name` function instead.
   */
  operator EntityRef() const;
};

auto operator==(const Entity &, const Entity &) -> bool;

auto operator<<(std::ostream &, const Entity &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

template <>
struct std::hash<openscenario_interpreter::Entity>
{
  auto operator()(const openscenario_interpreter::Entity & entity) const -> std::size_t
  {
    return std::hash<void *>{}(entity.get());
  }
};

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_HPP_
