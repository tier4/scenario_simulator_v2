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

#include <functional>
#include <iterator>
#include <numeric>
#include <openscenario_interpreter/object.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/object_type.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>
#include <set>
#include <type_traits>
#include <vector>

namespace openscenario_interpreter
{
class Scope;

inline namespace syntax
{
struct Entities;

struct Entity
{
private:
  EntityRef entity_ref;

  const Entities * entities;

public:
  template <typename Candidates>
  explicit Entity(const pugi::xml_node & node, Scope & scope, const Candidates & candidates)
  : Entity(EntityRef{node, scope, candidates}, scope)
  {
  }

  explicit Entity(EntityRef entity_ref, const Scope & scope);

  explicit Entity(EntityRef entity_ref, const Entities * entities);

  auto descendants() const -> std::set<EntityRef>;

  auto types() const -> std::set<ObjectType::value_type>;

  operator String() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_HPP_
