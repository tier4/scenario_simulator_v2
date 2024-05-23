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
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/collision_condition.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
CollisionCondition::CollisionCondition(
  const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
// clang-format off
: Scope(scope),
  another_given_entity(
    choice(node,
      std::make_pair("EntityRef", [&](auto && node) { return make<Entity>(node, scope); }),
      std::make_pair("ByType",    [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }))),
  triggering_entities(triggering_entities)
// clang-format on
{
}

auto CollisionCondition::description() const -> std::string
{
  std::stringstream description;

  description << triggering_entities.description() << " colliding with another given entity "
              << another_given_entity << "?";

  // TODO (yamacir-kit): If another_given_entity.is<ByType>(), description
  // will be "Is any of [A, B, C] colliding with another T typed entities?"

  return description.str();
}

auto CollisionCondition::evaluate() const -> Object
{
  if (
    another_given_entity.is<Entity>() and
    global().entities->isAdded(another_given_entity.as<Entity>())) {
    return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
      auto evaluation = triggering_entity.apply([&](const auto & object) {
        return evaluateCollisionCondition(object, another_given_entity.as<Entity>());
      });
      return not evaluation.size() or evaluation.min();
    }));
  } else {
    // TODO ByType
    return false_v;
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
