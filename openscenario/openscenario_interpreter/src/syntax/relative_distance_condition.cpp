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

//ignore spell miss due to OpenSCENARIO standard
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
  rule(readAttribute<Rule>("rule", node, scope)),
  value(readAttribute<Double>("value", node, scope)),
  triggering_entities(triggering_entities),
  results(triggering_entities.entity_refs.size(), Double::nan()),
  consider_z([]() {
    rclcpp::Node node{"get_parameter", "simulation"};
    node.declare_parameter("consider_pose_by_road_slope", false);
    return node.get_parameter("consider_pose_by_road_slope").as_bool();
  }())
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
  CoordinateSystem::entity, RelativeDistanceType::longitudinal, false>(
  const EntityRef & triggering_entity) -> double
{
  if (
    global().entities->at(triggering_entity).as<ScenarioObject>().is_added and
    global().entities->at(entity_ref).as<ScenarioObject>().is_added) {
    return std::abs(makeNativeRelativeWorldPosition(triggering_entity, entity_ref).position.x);
  } else {
    return Double::nan();
  }
}

/**
 * @note This implementation differs from the OpenSCENARIO standard. See the section "6.4. Distances" in the OpenSCENARIO User Guide.
 */
template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::longitudinal, true>(
  const EntityRef & triggering_entity) -> double
{
  if (
    global().entities->at(triggering_entity).as<ScenarioObject>().is_added and
    global().entities->at(entity_ref).as<ScenarioObject>().is_added) {
    return std::abs(
      makeNativeBoundingBoxRelativeWorldPosition(triggering_entity, entity_ref).position.x);
  } else {
    return Double::nan();
  }
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::lateral, false>(
  const EntityRef & triggering_entity) -> double
{
  if (
    global().entities->at(triggering_entity).as<ScenarioObject>().is_added and
    global().entities->at(entity_ref).as<ScenarioObject>().is_added) {
    return std::abs(makeNativeRelativeWorldPosition(triggering_entity, entity_ref).position.y);
  } else {
    return Double::nan();
  }
}

/**
 * @note This implementation differs from the OpenSCENARIO standard. See the section "6.4. Distances" in the OpenSCENARIO User Guide.
 */
template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::lateral, true>(
  const EntityRef & triggering_entity) -> double
{
  if (
    global().entities->at(triggering_entity).as<ScenarioObject>().is_added and
    global().entities->at(entity_ref).as<ScenarioObject>().is_added) {
    return std::abs(
      makeNativeBoundingBoxRelativeWorldPosition(triggering_entity, entity_ref).position.y);
  } else {
    return Double::nan();
  }
}

// @todo: after checking all the scenario work well with consider_z = true, remove this function and use std::hypot(x,y,z)
static double hypot(const double x, const double y, const double z, const bool consider_z)
{
  return consider_z ? std::hypot(x, y, z) : std::hypot(x, y);
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, true>(
  const EntityRef & triggering_entity) -> double
{
  if (
    global().entities->at(triggering_entity).as<ScenarioObject>().is_added and
    global().entities->at(entity_ref).as<ScenarioObject>().is_added) {
    return hypot(
      makeNativeBoundingBoxRelativeWorldPosition(triggering_entity, entity_ref).position.x,
      makeNativeBoundingBoxRelativeWorldPosition(triggering_entity, entity_ref).position.y,
      makeNativeBoundingBoxRelativeWorldPosition(triggering_entity, entity_ref).position.z,
      consider_z);
  } else {
    return Double::nan();
  }
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, false>(
  const EntityRef & triggering_entity) -> double
{
  if (
    global().entities->at(triggering_entity).as<ScenarioObject>().is_added and
    global().entities->at(entity_ref).as<ScenarioObject>().is_added) {
    return hypot(
      makeNativeRelativeWorldPosition(triggering_entity, entity_ref).position.x,
      makeNativeRelativeWorldPosition(triggering_entity, entity_ref).position.y,
      makeNativeRelativeWorldPosition(triggering_entity, entity_ref).position.z, consider_z);
  } else {
    return Double::nan();
  }
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, false>(const EntityRef & triggering_entity)
  -> double
{
  if (
    global().entities->at(entity_ref).as<ScenarioObject>().is_added and
    global().entities->at(triggering_entity).as<ScenarioObject>().is_added) {
    /*
       For historical reasons, signed distances are returned when
       coordinateSystem == lane and relativeDistanceType ==
       longitudinal/lateral. The sign has been mainly used to determine the
       front/back and left/right positional relationship (a negative value is
       returned if the target entity is behind or to the right).

       This behavior violates the OpenSCENARIO standard. In the future, after
       DistanceCondition and RelativeDistanceCondition of TIER IV's
       OpenSCENARIO Interpreter support OpenSCENARIO 1.2 RoutingAlgorithm, this
       behavior will be enabled only when routingAlgorithm == undefined.
    */
    return static_cast<traffic_simulator::LaneletPose>(
             makeNativeRelativeLanePosition(triggering_entity, entity_ref))
      .offset;
  } else {
    return Double::nan();
  }
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, true>(const EntityRef & triggering_entity)
  -> double
{
  if (
    global().entities->at(entity_ref).as<ScenarioObject>().is_added and
    global().entities->at(triggering_entity).as<ScenarioObject>().is_added) {
    /*
       For historical reasons, signed distances are returned when
       coordinateSystem == lane and relativeDistanceType ==
       longitudinal/lateral. The sign has been mainly used to determine the
       front/back and left/right positional relationship (a negative value is
       returned if the target entity is behind or to the right).

       This behavior violates the OpenSCENARIO standard. In the future, after
       DistanceCondition and RelativeDistanceCondition of TIER IV's
       OpenSCENARIO Interpreter support OpenSCENARIO 1.2 RoutingAlgorithm, this
       behavior will be enabled only when routingAlgorithm == undefined.
    */
    return static_cast<traffic_simulator::LaneletPose>(
             makeNativeBoundingBoxRelativeLanePosition(triggering_entity, entity_ref))
      .offset;
  } else {
    return Double::nan();
  }
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, false>(
  const EntityRef & triggering_entity) -> double
{
  if (
    global().entities->at(triggering_entity).as<ScenarioObject>().is_added and
    global().entities->at(entity_ref).as<ScenarioObject>().is_added) {
    /*
       For historical reasons, signed distances are returned when
       coordinateSystem == lane and relativeDistanceType ==
       longitudinal/lateral. The sign has been mainly used to determine the
       front/back and left/right positional relationship (a negative value is
       returned if the target entity is behind or to the right).

       This behavior violates the OpenSCENARIO standard. In the future, after
       DistanceCondition and RelativeDistanceCondition of TIER IV's
       OpenSCENARIO Interpreter support OpenSCENARIO 1.2 RoutingAlgorithm, this
       behavior will be enabled only when routingAlgorithm == undefined.
    */
    return static_cast<traffic_simulator::LaneletPose>(
             makeNativeRelativeLanePosition(triggering_entity, entity_ref))
      .s;
  } else {
    return Double::nan();
  }
}

template <>
auto RelativeDistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, true>(
  const EntityRef & triggering_entity) -> double
{
  if (
    global().entities->at(triggering_entity).as<ScenarioObject>().is_added and
    global().entities->at(entity_ref).as<ScenarioObject>().is_added) {
    /*
       For historical reasons, signed distances are returned when
       coordinateSystem == lane and relativeDistanceType ==
       longitudinal/lateral. The sign has been mainly used to determine the
       front/back and left/right positional relationship (a negative value is
       returned if the target entity is behind or to the right).

       This behavior violates the OpenSCENARIO standard. In the future, after
       DistanceCondition and RelativeDistanceCondition of TIER IV's
       OpenSCENARIO Interpreter support OpenSCENARIO 1.2 RoutingAlgorithm, this
       behavior will be enabled only when routingAlgorithm == undefined.
    */
    return static_cast<traffic_simulator::LaneletPose>(
             makeNativeBoundingBoxRelativeLanePosition(triggering_entity, entity_ref))
      .s;
  } else {
    return Double::nan();
  }
}

#define DISTANCE(...) distance<__VA_ARGS__>(triggering_entity)

#define SWITCH_FREESPACE(FUNCTION, ...) \
  return freespace ? FUNCTION(__VA_ARGS__, true) : FUNCTION(__VA_ARGS__, false)

#define SWITCH_RELATIVE_DISTANCE_TYPE(FUNCTION, ...)                  \
  switch (relative_distance_type) {                                   \
    case RelativeDistanceType::longitudinal:                          \
      FUNCTION(__VA_ARGS__, RelativeDistanceType::longitudinal);      \
      break;                                                          \
    case RelativeDistanceType::lateral:                               \
      FUNCTION(__VA_ARGS__, RelativeDistanceType::lateral);           \
      break;                                                          \
    case RelativeDistanceType::euclidianDistance:                     \
      FUNCTION(__VA_ARGS__, RelativeDistanceType::euclidianDistance); \
      break;                                                          \
  }

#define SWITCH_COORDINATE_SYSTEM(FUNCTION, ...)            \
  switch (coordinate_system) {                             \
    case CoordinateSystem::entity:                         \
      FUNCTION(__VA_ARGS__, CoordinateSystem::entity);     \
      break;                                               \
    case CoordinateSystem::lane:                           \
      FUNCTION(__VA_ARGS__, CoordinateSystem::lane);       \
      break;                                               \
    case CoordinateSystem::road:                           \
      FUNCTION(__VA_ARGS__, CoordinateSystem::road);       \
      break;                                               \
    case CoordinateSystem::trajectory:                     \
      FUNCTION(__VA_ARGS__, CoordinateSystem::trajectory); \
      break;                                               \
  }

#define APPLY(F, ...) F(__VA_ARGS__)

auto RelativeDistanceCondition::distance(const EntityRef & triggering_entity) -> double
{
  APPLY(SWITCH_COORDINATE_SYSTEM, SWITCH_RELATIVE_DISTANCE_TYPE, SWITCH_FREESPACE, DISTANCE);
  return Double::nan();
}

auto RelativeDistanceCondition::evaluate() -> Object
{
  results.clear();

  return asBoolean(triggering_entities.apply([&](const auto & triggering_entity) {
    results.push_back(distance(triggering_entity));
    return rule(static_cast<double>(results.back()), value);
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
