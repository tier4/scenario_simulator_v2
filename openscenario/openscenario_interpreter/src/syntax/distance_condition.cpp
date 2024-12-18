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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/distance_condition.hpp>
#include <openscenario_interpreter/syntax/entities.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/utility/overload.hpp>
#include <openscenario_interpreter/utility/print.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

// NOTE: Ignore spell miss due to OpenSCENARIO standard.
// cspell: ignore euclidian

namespace openscenario_interpreter
{
inline namespace syntax
{
DistanceCondition::DistanceCondition(
  const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
: Scope(scope),
  coordinate_system(
    readAttribute<CoordinateSystem>("coordinateSystem", node, scope, CoordinateSystem::entity)),
  freespace(readAttribute<Boolean>("freespace", node, scope)),
  relative_distance_type(readAttribute<RelativeDistanceType>(
    "relativeDistanceType", node, scope, RelativeDistanceType::euclidianDistance)),
  routing_algorithm(
    readAttribute<RoutingAlgorithm>("routingAlgorithm", node, scope, RoutingAlgorithm::undefined)),
  rule(readAttribute<Rule>("rule", node, scope)),
  value(readAttribute<Double>("value", node, scope)),
  position(readElement<Position>("Position", node, scope)),
  triggering_entities(triggering_entities),
  results(triggering_entities.entity_refs.size(), {Double::nan()})
{
  std::set<RoutingAlgorithm::value_type> supported = {
    RoutingAlgorithm::value_type::shortest, RoutingAlgorithm::value_type::undefined};
  if (supported.find(routing_algorithm) == supported.end()) {
    throw UNSUPPORTED_ENUMERATION_VALUE_SPECIFIED(
      DistanceCondition, boost::lexical_cast<std::string>(routing_algorithm));
  }
}

auto DistanceCondition::description() const -> std::string
{
  std::stringstream description;

  description << triggering_entities.description() << "'s distance to given position = ";

  print_to(description, results);

  description << " " << rule << " " << value << "?";

  return description.str();
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

#define DISTANCE(...) distance<__VA_ARGS__>(triggering_entity, position)

auto DistanceCondition::evaluate(
  const Entities * entities, const Entity & triggering_entity, const Position & position,
  CoordinateSystem coordinate_system, RelativeDistanceType relative_distance_type,
  RoutingAlgorithm routing_algorithm, Boolean freespace) -> double
{
  if (entities->isAdded(triggering_entity)) {
    SWITCH_COORDINATE_SYSTEM(
      SWITCH_RELATIVE_DISTANCE_TYPE, SWITCH_ROUTING_ALGORITHM, SWITCH_FREESPACE, DISTANCE);
  } else {
    return Double::nan();
  }
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, RoutingAlgorithm::undefined,
  false>(const EntityRef & triggering_entity, const Position & position) -> double
{
  return euclideanDistance(triggering_entity, static_cast<NativeWorldPosition>(position));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, RoutingAlgorithm::undefined,
  true>(const EntityRef & triggering_entity, const Position & position) -> double
{
  return euclideanBoundingBoxDistance(
    triggering_entity, static_cast<NativeWorldPosition>(position));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return lateralEntityDistance(triggering_entity, static_cast<NativeWorldPosition>(position));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return lateralEntityBoundingBoxDistance(
    triggering_entity, static_cast<NativeWorldPosition>(position));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return longitudinalEntityDistance(triggering_entity, static_cast<NativeWorldPosition>(position));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return longitudinalEntityBoundingBoxDistance(
    triggering_entity, static_cast<NativeWorldPosition>(position));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return lateralLaneDistance(triggering_entity, static_cast<NativeLanePosition>(position));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return lateralLaneBoundingBoxDistance(
    triggering_entity, static_cast<NativeLanePosition>(position));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return longitudinalLaneDistance(triggering_entity, static_cast<NativeLanePosition>(position));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return longitudinalLaneBoundingBoxDistance(
    triggering_entity, static_cast<NativeLanePosition>(position));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::shortest, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return std::abs(lateralLaneDistance(
    triggering_entity, static_cast<NativeLanePosition>(position), RoutingAlgorithm::shortest));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::shortest, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return std::abs(lateralLaneBoundingBoxDistance(
    triggering_entity, static_cast<NativeLanePosition>(position), RoutingAlgorithm::shortest));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::shortest, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return std::abs(longitudinalLaneDistance(
    triggering_entity, static_cast<NativeLanePosition>(position), RoutingAlgorithm::shortest));
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::shortest, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return std::abs(longitudinalLaneBoundingBoxDistance(
    triggering_entity, static_cast<NativeLanePosition>(position), RoutingAlgorithm::shortest));
}

auto DistanceCondition::evaluate() -> Object
{
  results.clear();

  return asBoolean(triggering_entities.apply([&](const auto & triggering_entity) {
    results.push_back(triggering_entity.apply([&](const auto & triggering_entity) {
      return evaluate(
        global().entities, triggering_entity, position, coordinate_system, relative_distance_type,
        routing_algorithm, freespace);
    }));
    return not results.back().size() or rule(results.back(), value).min();
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
