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

#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/entity_selection.hpp>
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
  entity_ref(readAttribute<String>("entityRef", node, scope), scope),
  rule(readAttribute<Rule>("rule", node, scope)),
  value(readAttribute<Double>("value", node, scope)),
  direction(readAttribute<DirectionalDimension>("direction", node, scope, std::nullopt)),
  triggering_entities(triggering_entities),
  evaluations(triggering_entities.entity_refs.size(), {Double::nan()})
{
  if (entity_ref.is<EntitySelection>()) {
    throw SyntaxError("EntitySelection is not allowed in RelativeSpeedCondition.entityRef");
  }
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
  const Entities * entities, const Entity & triggering_entity, const Entity & entity_ref)
  -> Eigen::Vector3d
{
  if (
    triggering_entity.apply([&](const auto & each) { return entities->isAdded(each); }).min() and
    entities->isAdded(entity_ref)) {
    /*
       Relative speed is defined as speed_rel = speed(triggering entity) -
       speed(reference entity). In other words, entity_ref is the observer and
       triggering_entity is the observed.

       See: https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/RelativeSpeedCondition.html
    */
    return evaluateRelativeSpeed(entity_ref, triggering_entity);
  } else {
    return Eigen::Vector3d(Double::nan(), Double::nan(), Double::nan());
  }
}

auto RelativeSpeedCondition::evaluate(
  const Entities * entities, const Entity & triggering_entity, const Entity & entity_ref,
  const std::optional<DirectionalDimension> & direction) -> double
{
  if (const Eigen::Vector3d v = evaluate(entities, triggering_entity, entity_ref); direction) {
    switch (*direction) {
      default:
      case DirectionalDimension::longitudinal:
        return v.x();
      case DirectionalDimension::lateral:
        return v.y();
      case DirectionalDimension::vertical:
        return v.z();
    }
  } else {
    return v.norm();
  }
}

auto RelativeSpeedCondition::evaluate() -> Object
{
  evaluations.clear();

  return asBoolean(triggering_entities.apply([this](const auto & triggering_entity) {
    evaluations.push_back(triggering_entity.apply([this](const auto & triggering_entity) {
      return evaluate(global().entities, triggering_entity, entity_ref, direction);
    }));
    return not evaluations.back().size() or std::invoke(rule, evaluations.back(), value).min();
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
