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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TIME_TO_COLLISION_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TIME_TO_COLLISION_CONDITION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/distance_condition.hpp>
#include <openscenario_interpreter/syntax/relative_distance_condition.hpp>
#include <openscenario_interpreter/syntax/relative_speed_condition.hpp>
#include <openscenario_interpreter/syntax/speed_condition.hpp>
#include <openscenario_interpreter/syntax/time_to_collision_condition_target.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   TimeToCollisionCondition (OpenSCENARIO XML 1.3.1)

   The currently predicted time to collision of a triggering entity/entities
   and either a reference entity or an explicit position is compared to a given
   time value. Time to collision is calculated as the distance divided by the
   relative speed. Acceleration of entities is not taken into consideration.
   For the relative speed calculation the same coordinate system and relative
   distance type applies as for the distance calculation. If the relative speed
   is negative, which means the entity is moving away from the position / the
   entities are moving away from each other, then the time to collision cannot
   be predicted and the condition evaluates to false. The logical operator for
   comparison is defined by the rule attribute. The property "alongRoute" is
   deprecated. If "coordinateSystem" or "relativeDistanceType" are set,
   "alongRoute" is ignored.

   <xsd:complexType name="TimeToCollisionCondition">
     <xsd:all>
       <xsd:element name="TimeToCollisionConditionTarget" type="TimeToCollisionConditionTarget"/>
     </xsd:all>
     <xsd:attribute name="alongRoute" type="Boolean">
       <xsd:annotation>
         <xsd:appinfo>
           deprecated
         </xsd:appinfo>
       </xsd:annotation>
     </xsd:attribute>
     <xsd:attribute name="freespace" type="Boolean" use="required"/>
     <xsd:attribute name="rule" type="Rule" use="required"/>
     <xsd:attribute name="value" type="Double" use="required"/>
     <xsd:attribute name="relativeDistanceType" type="RelativeDistanceType"/>
     <xsd:attribute name="coordinateSystem" type="CoordinateSystem"/>
     <xsd:attribute name="routingAlgorithm" type="RoutingAlgorithm"/>
   </xsd:complexType>
*/
struct TimeToCollisionCondition : private Scope, private SimulatorCore::ConditionEvaluation
{
  const TimeToCollisionConditionTarget time_to_collision_condition_target;

  const Boolean freespace;

  const Rule rule;

  const Double value;

  const RelativeDistanceType relative_distance_type;

  const CoordinateSystem coordinate_system;

  const RoutingAlgorithm routing_algorithm;

  const TriggeringEntities triggering_entities;

  std::vector<std::valarray<double>> evaluations;

  explicit TimeToCollisionCondition(
    const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
  : Scope(scope),
    time_to_collision_condition_target(
      readElement<TimeToCollisionConditionTarget>("TimeToCollisionConditionTarget", node, scope)),
    freespace(readAttribute<Boolean>("freespace", node, scope)),
    rule(readAttribute<Rule>("rule", node, scope)),
    value(readAttribute<Double>("value", node, scope)),
    relative_distance_type(
      readAttribute<RelativeDistanceType>("relativeDistanceType", node, scope)),
    coordinate_system(
      readAttribute<CoordinateSystem>("coordinateSystem", node, scope, CoordinateSystem::entity)),
    routing_algorithm(readAttribute<RoutingAlgorithm>(
      "routingAlgorithm", node, scope, RoutingAlgorithm::undefined)),
    triggering_entities(triggering_entities),
    evaluations(triggering_entities.entity_refs.size(), {Double::nan()})
  {
  }

  auto description() const
  {
    auto description = std::stringstream();

    description << triggering_entities.description() << "'s time-to-collision to given ";

    if (time_to_collision_condition_target.is<Entity>()) {
      description << " entity " << time_to_collision_condition_target.as<Entity>();
    } else {
      description << " position";
    }

    description << " = ";

    print_to(description, evaluations);

    description << " " << rule << " " << value << "?";

    return description.str();
  }

  static auto evaluate(
    const Entities * entities, const Entity & triggering_entity,
    const TimeToCollisionConditionTarget & time_to_collision_condition_target,
    CoordinateSystem coordinate_system, RelativeDistanceType relative_distance_type,
    RoutingAlgorithm routing_algorithm, Boolean freespace)
  {
    auto distance = [&]() {
      if (time_to_collision_condition_target.is<Entity>()) {
        return RelativeDistanceCondition::evaluate(
          entities, triggering_entity, time_to_collision_condition_target.as<Entity>(),
          coordinate_system, relative_distance_type, routing_algorithm, freespace);
      } else {
        return DistanceCondition::evaluate(
          entities, triggering_entity, time_to_collision_condition_target.as<Position>(),
          coordinate_system, relative_distance_type, routing_algorithm, freespace);
      }
    };

    auto speed = [&]() {
      auto directional_dimension = [&]() {
        switch (relative_distance_type) {
          case RelativeDistanceType::longitudinal:
            return std::optional<DirectionalDimension>(DirectionalDimension::longitudinal);
          case RelativeDistanceType::lateral:
            return std::optional<DirectionalDimension>(DirectionalDimension::lateral);
          default:
          case RelativeDistanceType::euclidianDistance:
            return std::optional<DirectionalDimension>(std::nullopt);
        };
      };
      if (time_to_collision_condition_target.is<Entity>()) {
        return RelativeSpeedCondition::evaluate(
          entities, triggering_entity, time_to_collision_condition_target.as<Entity>(),
          directional_dimension());
      } else {
        return SpeedCondition::evaluate(
          entities, triggering_entity, directional_dimension(), Compatibility::standard);
      }
    };

    return distance() / std::max(speed(), 0.0);
  }

  auto evaluate()
  {
    evaluations.clear();

    return asBoolean(triggering_entities.apply([this](const auto & triggering_entity) {
      evaluations.push_back(triggering_entity.apply([this](const auto & triggering_entity) {
        return evaluate(
          global().entities, triggering_entity, time_to_collision_condition_target,
          coordinate_system, relative_distance_type, routing_algorithm, freespace);
      }));
      return not evaluations.back().size() or std::invoke(rule, evaluations.back(), value).min();
    }));
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TIME_TO_COLLISION_CONDITION_HPP_
