// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/collision_condition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto CollisionCondition::description() const -> std::string
{
  std::stringstream description;

  description << triggering_entities.description() << " colliding with another given entity "
              << another_given_entity << "?";

  // TODO (yamacir-kit): If another_given_entity.is<ByType>(), description
  // will be "Is any of [A, B, C] colliding with another T typed entities?"

  return description.str();
}

auto CollisionCondition::evaluate() const -> Element
{
  if (
    another_given_entity.is<EntityRef>() and
    global().isAddedEntity(another_given_entity.as<EntityRef>())) {
    return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
      return evaluateCollisionCondition(triggering_entity, another_given_entity.as<EntityRef>());
    }));
  } else {
    // TODO ByType
    return false_v;
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
