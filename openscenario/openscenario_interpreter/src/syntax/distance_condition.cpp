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

#include <cmath>
#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/distance_condition.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/utility/overload.hpp>
#include <openscenario_interpreter/utility/print.hpp>
#include <sstream>

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
  rule(readAttribute<Rule>("rule", node, scope)),
  value(readAttribute<Double>("value", node, scope)),
  position(readElement<Position>("Position", node, scope)),
  triggering_entities(triggering_entities),
  results(triggering_entities.entity_refs.size(), Double::nan())
{
}

auto DistanceCondition::description() const -> std::string
{
  std::stringstream description;

  description << triggering_entities.description() << "'s distance to given position = ";

  print_to(description, results);

  description << " " << rule << " " << value << "?";

  return description.str();
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

auto DistanceCondition::distance(const EntityRef & triggering_entity) const -> double
{
  APPLY(SWITCH_COORDINATE_SYSTEM, SWITCH_RELATIVE_DISTANCE_TYPE, SWITCH_FREESPACE, DISTANCE);
  return std::numeric_limits<double>::quiet_NaN();
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, false>(
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        const auto pose =
          getRelativePose(triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
        return std::hypot(pose.position.x, pose.position.y);
      },
      [&](const RelativeWorldPosition & position) {
        const auto pose =
          getRelativePose(triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
        return std::hypot(pose.position.x, pose.position.y);
      },
      [&](const LanePosition & position) {
        const auto pose =
          getRelativePose(triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
        return std::hypot(pose.position.x, pose.position.y);
      }),
    position);
}

template <>
auto DistanceCondition::distance<  //
  CoordinateSystem::entity, RelativeDistanceType::lateral, false>(
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return getRelativePose(triggering_entity, static_cast<geometry_msgs::msg::Pose>(position))
          .position.y;
      },
      [&](const RelativeWorldPosition & position) {
        return getRelativePose(triggering_entity, static_cast<geometry_msgs::msg::Pose>(position))
          .position.y;
      },
      [&](const LanePosition & position) {
        return getRelativePose(triggering_entity, static_cast<geometry_msgs::msg::Pose>(position))
          .position.y;
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::longitudinal, false>(
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return getRelativePose(triggering_entity, static_cast<geometry_msgs::msg::Pose>(position))
          .position.x;
      },
      [&](const RelativeWorldPosition & position) {
        return getRelativePose(triggering_entity, static_cast<geometry_msgs::msg::Pose>(position))
          .position.x;
      },
      [&](const LanePosition & position) {
        return getRelativePose(triggering_entity, static_cast<geometry_msgs::msg::Pose>(position))
          .position.x;
      }),
    position);
}

template <>
auto DistanceCondition::distance<  //
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, false>(
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        if (global().entityRef(triggering_entity).as<ScenarioObject>().is_added) {
          return getLongitudinalDistance(
            triggering_entity, static_cast<traffic_simulator_msgs::msg::LaneletPose>(position));
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeWorldPosition & position) {
        if (global().entityRef(triggering_entity).as<ScenarioObject>().is_added) {
          return getLongitudinalDistance(
            triggering_entity, static_cast<traffic_simulator_msgs::msg::LaneletPose>(position));
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const LanePosition & position) {
        if (global().entityRef(triggering_entity).as<ScenarioObject>().is_added) {
          return getLongitudinalDistance(
            triggering_entity, static_cast<traffic_simulator_msgs::msg::LaneletPose>(position));
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      }),
    position);
}

auto DistanceCondition::evaluate() -> Object
{
  results.clear();

  return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
    results.push_back(distance(triggering_entity));
    return rule(results.back(), value);
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
