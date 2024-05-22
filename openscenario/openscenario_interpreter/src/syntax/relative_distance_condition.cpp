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
#include <openscenario_interpreter/syntax/entities.hpp>  // TEMPORARY (TODO REMOVE THIS LINE)
#include <openscenario_interpreter/syntax/relative_distance_condition.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/utility/print.hpp>

// Ignore spell miss due to OpenSCENARIO standard.
// cspell: ignore euclidian

namespace openscenario_interpreter
{
inline namespace syntax
{
RelativeDistanceCondition::RelativeDistanceCondition(
  const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
: Scope(scope),
  coordinate_system(
    readAttribute<CoordinateSystem>("coordinateSystem", node, scope, CoordinateSystem::entity)),
  entity_ref(readAttribute<String>("entityRef", node, scope)),
  freespace(readAttribute<Boolean>("freespace", node, scope)),
  relative_distance_type(readAttribute<RelativeDistanceType>("relativeDistanceType", node, scope)),
  routing_algorithm(
    readAttribute<RoutingAlgorithm>("routingAlgorithm", node, scope, RoutingAlgorithm::undefined)),
  rule(readAttribute<Rule>("rule", node, scope)),
  value(readAttribute<Double>("value", node, scope)),
  triggering_entities(triggering_entities),
  results(triggering_entities.entity_refs.size(), Double::nan())
{
}

auto RelativeDistanceCondition::description() const -> String
{
  std::stringstream description;

  description << triggering_entities.description() << "'s relative distance to given entity "
              << entity_ref << " = ";

  print_to(description, results);

  description << " " << rule << " " << value << "?";

  return description.str();
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  return std::abs(makeNativeRelativeWorldPosition(triggering_entity, entity_ref).position.x);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  /**
     @note This implementation differs from the OpenSCENARIO standard. See the
     section "6.4. Distances" in the OpenSCENARIO User Guide.
  */
  return std::abs(
    makeNativeBoundingBoxRelativeWorldPosition(triggering_entity, entity_ref).position.x);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  return std::abs(makeNativeRelativeWorldPosition(triggering_entity, entity_ref).position.y);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  /**
     @note This implementation differs from the OpenSCENARIO standard. See the
     section "6.4. Distances" in the OpenSCENARIO User Guide.
  */
  return std::abs(
    makeNativeBoundingBoxRelativeWorldPosition(triggering_entity, entity_ref).position.y);
}

// @todo: after checking all the scenario work well with consider_z = true, remove this function and use std::hypot(x, y, z)
auto hypot(double x, double y, double z)
{
  static auto consider_z = []() {
    auto node = rclcpp::Node("get_parameter", "simulation");
    node.declare_parameter("consider_pose_by_road_slope", false);
    return node.get_parameter("consider_pose_by_road_slope").as_bool();
  }();

  return consider_z ? std::hypot(x, y, z) : std::hypot(x, y);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, RoutingAlgorithm::undefined,
  true>(const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  const auto relative_world =
    makeNativeBoundingBoxRelativeWorldPosition(triggering_entity, entity_ref);
  return hypot(relative_world.position.x, relative_world.position.y, relative_world.position.z);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, RoutingAlgorithm::undefined,
  false>(const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  const auto relative_world = makeNativeRelativeWorldPosition(triggering_entity, entity_ref);
  return hypot(relative_world.position.x, relative_world.position.y, relative_world.position.z);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  /*
     For historical reasons, signed distances are returned when
     coordinateSystem == lane and relativeDistanceType == longitudinal/lateral.
     The sign has been mainly used to determine the front/back and left/right
     positional relationship (a negative value is returned if the target entity
     is behind or to the right).

     This behavior violates the OpenSCENARIO standard. In the future, after
     DistanceCondition and RelativeDistanceCondition of TIER IV's OpenSCENARIO
     Interpreter support OpenSCENARIO 1.2 RoutingAlgorithm, this behavior will
     be enabled only when routingAlgorithm == undefined.
  */
  return static_cast<traffic_simulator::LaneletPose>(
           makeNativeRelativeLanePosition(triggering_entity, entity_ref))
    .offset;
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  /*
     For historical reasons, signed distances are returned when
     coordinateSystem == lane and relativeDistanceType == longitudinal/lateral.
     The sign has been mainly used to determine the front/back and left/right
     positional relationship (a negative value is returned if the target entity
     is behind or to the right).

     This behavior violates the OpenSCENARIO standard. In the future, after
     DistanceCondition and RelativeDistanceCondition of TIER IV's OpenSCENARIO
     Interpreter support OpenSCENARIO 1.2 RoutingAlgorithm, this behavior will
     be enabled only when routingAlgorithm == undefined.
  */
  return static_cast<traffic_simulator::LaneletPose>(
           makeNativeBoundingBoxRelativeLanePosition(triggering_entity, entity_ref))
    .offset;
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  /*
     For historical reasons, signed distances are returned when
     coordinateSystem == lane and relativeDistanceType == longitudinal/lateral.
     The sign has been mainly used to determine the front/back and left/right
     positional relationship (a negative value is returned if the target entity
     is behind or to the right).

     This behavior violates the OpenSCENARIO standard. In the future, after
     DistanceCondition and RelativeDistanceCondition of TIER IV's OpenSCENARIO
     Interpreter support OpenSCENARIO 1.2 RoutingAlgorithm, this behavior will
     be enabled only when routingAlgorithm == undefined.
  */
  return static_cast<traffic_simulator::LaneletPose>(
           makeNativeRelativeLanePosition(triggering_entity, entity_ref))
    .s;
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  /*
     For historical reasons, signed distances are returned when
     coordinateSystem == lane and relativeDistanceType == longitudinal/lateral.
     The sign has been mainly used to determine the front/back and left/right
     positional relationship (a negative value is returned if the target entity
     is behind or to the right).

     This behavior violates the OpenSCENARIO standard. In the future, after
     DistanceCondition and RelativeDistanceCondition of TIER IV's OpenSCENARIO
     Interpreter support OpenSCENARIO 1.2 RoutingAlgorithm, this behavior will
     be enabled only when routingAlgorithm == undefined.
  */
  return static_cast<traffic_simulator::LaneletPose>(
           makeNativeBoundingBoxRelativeLanePosition(triggering_entity, entity_ref))
    .s;
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::shortest, true>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  return std::abs(static_cast<traffic_simulator::LaneletPose>(
                    makeNativeBoundingBoxRelativeLanePosition(
                      triggering_entity, entity_ref, RoutingAlgorithm::shortest))
                    .offset);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::shortest, false>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  return std::abs(
    static_cast<traffic_simulator::LaneletPose>(
      makeNativeRelativeLanePosition(triggering_entity, entity_ref, RoutingAlgorithm::shortest))
      .offset);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::shortest, true>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  return std::abs(static_cast<traffic_simulator::LaneletPose>(
                    makeNativeBoundingBoxRelativeLanePosition(
                      triggering_entity, entity_ref, RoutingAlgorithm::shortest))
                    .s);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::shortest, false>(
  const EntityRef & triggering_entity, const EntityRef & entity_ref) -> double
{
  return std::abs(
    static_cast<traffic_simulator::LaneletPose>(
      makeNativeRelativeLanePosition(triggering_entity, entity_ref, RoutingAlgorithm::shortest))
      .s);
}

#define SWITCH_COORDINATE_SYSTEM(FUNCTION, ...)                                         \
  switch (coordinate_system) {                                                          \
    case CoordinateSystem::entity:                                                      \
      FUNCTION(__VA_ARGS__, CoordinateSystem::entity);                                  \
      break;                                                                            \
    case CoordinateSystem::lane:                                                        \
      FUNCTION(__VA_ARGS__, CoordinateSystem::lane);                                    \
      break;                                                                            \
    case CoordinateSystem::road:                                                        \
      FUNCTION(__VA_ARGS__, CoordinateSystem::road);                                    \
      break;                                                                            \
    case CoordinateSystem::trajectory:                                                  \
      FUNCTION(__VA_ARGS__, CoordinateSystem::trajectory);                              \
      break;                                                                            \
    default:                                                                            \
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(CoordinateSystem, coordinate_system); \
  }

#define SWITCH_RELATIVE_DISTANCE_TYPE(FUNCTION, ...)                                             \
  switch (relative_distance_type) {                                                              \
    case RelativeDistanceType::longitudinal:                                                     \
      FUNCTION(__VA_ARGS__, RelativeDistanceType::longitudinal);                                 \
      break;                                                                                     \
    case RelativeDistanceType::lateral:                                                          \
      FUNCTION(__VA_ARGS__, RelativeDistanceType::lateral);                                      \
      break;                                                                                     \
    case RelativeDistanceType::euclidianDistance:                                                \
      FUNCTION(__VA_ARGS__, RelativeDistanceType::euclidianDistance);                            \
      break;                                                                                     \
    default:                                                                                     \
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(RelativeDistanceType, relative_distance_type); \
  }

#define SWITCH_ROUTING_ALGORITHM(FUNCTION, ...)                                         \
  switch (routing_algorithm) {                                                          \
    case RoutingAlgorithm::assigned_route:                                              \
      FUNCTION(__VA_ARGS__, RoutingAlgorithm::assigned_route);                          \
      break;                                                                            \
    case RoutingAlgorithm::fastest:                                                     \
      FUNCTION(__VA_ARGS__, RoutingAlgorithm::fastest);                                 \
      break;                                                                            \
    case RoutingAlgorithm::least_intersections:                                         \
      FUNCTION(__VA_ARGS__, RoutingAlgorithm::least_intersections);                     \
      break;                                                                            \
    case RoutingAlgorithm::shortest:                                                    \
      FUNCTION(__VA_ARGS__, RoutingAlgorithm::shortest);                                \
      break;                                                                            \
    case RoutingAlgorithm::undefined:                                                   \
      FUNCTION(__VA_ARGS__, RoutingAlgorithm::undefined);                               \
      break;                                                                            \
    default:                                                                            \
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(RoutingAlgorithm, routing_algorithm); \
  }

#define SWITCH_FREESPACE(FUNCTION, ...) \
  return freespace ? FUNCTION(__VA_ARGS__, true) : FUNCTION(__VA_ARGS__, false)

#define DISTANCE(...) distance<__VA_ARGS__>(triggering_entity, entity_ref)

auto RelativeDistanceCondition::distance(
  const EntityRef & triggering_entity, const EntityRef & entity_ref, const Entities & entities,
  CoordinateSystem coordinate_system, RelativeDistanceType relative_distance_type,
  RoutingAlgorithm routing_algorithm, bool freespace) -> double
{
  if (
    entities.at(triggering_entity).as<ScenarioObject>().is_added and
    entities.at(entity_ref).as<ScenarioObject>().is_added) {
    SWITCH_COORDINATE_SYSTEM(
      SWITCH_RELATIVE_DISTANCE_TYPE, SWITCH_ROUTING_ALGORITHM, SWITCH_FREESPACE, DISTANCE);
  } else {
    return Double::nan();
  }
}

auto RelativeDistanceCondition::evaluate() -> Object
{
  results.clear();

  return asBoolean(triggering_entities.apply([&](const auto & triggering_entity) {
    results.push_back(distance(
      triggering_entity, entity_ref, *global().entities, coordinate_system, relative_distance_type,
      routing_algorithm, freespace));
    return rule(static_cast<double>(results.back()), value);
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
