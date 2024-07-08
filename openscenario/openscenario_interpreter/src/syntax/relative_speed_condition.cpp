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

#include <geometry/vector3/norm.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/relative_speed_condition.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
RelativeSpeedCondition::RelativeSpeedCondition(
  const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
: Scope(scope),
  entity_ref(readAttribute<EntityRef>("entityRef", node, scope)),
  rule(readAttribute<Rule>("rule", node, scope)),
  value(readAttribute<Double>("value", node, scope)),
  direction(readAttribute<DirectionalDimension>("direction", node, scope, std::nullopt)),
  triggering_entities(triggering_entities),
  evaluations(triggering_entities.entity_refs.size(), Double::nan())
{
}

auto RelativeSpeedCondition::description() const -> String
{
  auto description = std::stringstream();

  description << triggering_entities.description() << "'s relative ";

  if (direction) {
    description << *direction << " ";
  }

  description << "speed to given entity " << entity_ref << " = ";

  print_to(description, evaluations);

  description << " " << rule << " " << value << "?";

  return description.str();
}

auto RelativeSpeedCondition::evaluate(
  const Entities * entities, const EntityRef & triggering_entity, const EntityRef & entity_ref,
  const std::optional<DirectionalDimension> & direction) -> double
{
  if (entities->isAdded(triggering_entity) and entities->isAdded(entity_ref)) {
    if (const auto v = evaluateRelativeSpeed(triggering_entity, entity_ref); direction) {
      switch (*direction) {
        case DirectionalDimension::longitudinal:
          return v.x;
        case DirectionalDimension::lateral:
          return v.y;
        case DirectionalDimension::vertical:
          return v.z;
        default:
          return math::geometry::norm(v);
      }
    } else {
      return math::geometry::norm(v);
    }
  } else {
    return Double::nan();
  }
}

auto RelativeSpeedCondition::evaluate() -> Object
{
  evaluations.clear();

  return asBoolean(triggering_entities.apply([this](auto && triggering_entity) {
    evaluations.push_back(evaluate(global().entities, triggering_entity, entity_ref, direction));
    return std::invoke(rule, evaluations.back(), value);
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
