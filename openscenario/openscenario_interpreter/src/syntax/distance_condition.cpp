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

#include <openscenario_interpreter/cmath/hypot.hpp>
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
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        const auto relative_world = makeNativeRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z);
      },
      [&](const RelativeWorldPosition & position) {
        const auto relative_world = makeNativeRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z);
      },
      [&](const RelativeObjectPosition & position) {
        const auto relative_world = makeNativeRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z);
      },
      [&](const LanePosition & position) {
        const auto relative_world = makeNativeRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z);
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, RoutingAlgorithm::undefined,
  true>(const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        const auto relative_world = makeNativeBoundingBoxRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z);
      },
      [&](const RelativeWorldPosition & position) {
        const auto relative_world = makeNativeBoundingBoxRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z);
      },
      [&](const RelativeObjectPosition & position) {
        const auto relative_world = makeNativeBoundingBoxRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z);
      },
      [&](const LanePosition & position) {
        const auto relative_world = makeNativeBoundingBoxRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z);
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return makeNativeRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.y;
      },
      [&](const RelativeWorldPosition & position) {
        return makeNativeRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.y;
      },
      [&](const RelativeObjectPosition & position) {
        return makeNativeRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.y;
      },
      [&](const LanePosition & position) {
        return makeNativeRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.y;
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return makeNativeBoundingBoxRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.y;
      },
      [&](const RelativeWorldPosition & position) {
        return makeNativeBoundingBoxRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.y;
      },
      [&](const RelativeObjectPosition & position) {
        return makeNativeBoundingBoxRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.y;
      },
      [&](const LanePosition & position) {
        return makeNativeBoundingBoxRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.y;
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return makeNativeRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.x;
      },
      [&](const RelativeWorldPosition & position) {
        return makeNativeRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.x;
      },
      [&](const RelativeObjectPosition & position) {
        return makeNativeRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.x;
      },
      [&](const LanePosition & position) {
        return makeNativeRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.x;
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return makeNativeBoundingBoxRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.x;
      },
      [&](const RelativeWorldPosition & position) {
        return makeNativeBoundingBoxRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.x;
      },
      [&](const RelativeObjectPosition & position) {
        return makeNativeBoundingBoxRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.x;
      },
      [&](const LanePosition & position) {
        return makeNativeBoundingBoxRelativeWorldPosition(
                 triggering_entity, static_cast<NativeWorldPosition>(position))
          .position.x;
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .offset;
      },
      [&](const RelativeWorldPosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .offset;
      },
      [&](const RelativeObjectPosition & position) {
        return makeNativeRelativeLanePosition(
                 triggering_entity, static_cast<NativeLanePosition>(position))
          .offset;
      },
      [&](const LanePosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .offset;
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeBoundingBoxRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .offset;
      },
      [&](const RelativeWorldPosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeBoundingBoxRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .offset;
      },
      [&](const RelativeObjectPosition & position) {
        return makeNativeBoundingBoxRelativeLanePosition(
                 triggering_entity, static_cast<NativeLanePosition>(position))
          .offset;
      },
      [&](const LanePosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeBoundingBoxRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .offset;
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .s;
      },
      [&](const RelativeWorldPosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .s;
      },
      [&](const RelativeObjectPosition & position) {
        return makeNativeRelativeLanePosition(
                 triggering_entity, static_cast<NativeLanePosition>(position))
          .s;
      },
      [&](const LanePosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .s;
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeBoundingBoxRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .s;
      },
      [&](const RelativeWorldPosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeBoundingBoxRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .s;
      },
      [&](const RelativeObjectPosition & position) {
        return makeNativeBoundingBoxRelativeLanePosition(
                 triggering_entity, static_cast<NativeLanePosition>(position))
          .s;
      },
      [&](const LanePosition & position) {
        return static_cast<traffic_simulator::LaneletPose>(
                 makeNativeBoundingBoxRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position)))
          .s;
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::shortest, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .offset);
      },
      [&](const RelativeWorldPosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .offset);
      },
      [&](const RelativeObjectPosition & position) {
        return std::abs(makeNativeRelativeLanePosition(
                          triggering_entity, static_cast<NativeLanePosition>(position),
                          RoutingAlgorithm::shortest)
                          .offset);
      },
      [&](const LanePosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .offset);
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::shortest, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeBoundingBoxRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .offset);
      },
      [&](const RelativeWorldPosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeBoundingBoxRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .offset);
      },
      [&](const RelativeObjectPosition & position) {
        return std::abs(makeNativeBoundingBoxRelativeLanePosition(
                          triggering_entity, static_cast<NativeLanePosition>(position),
                          RoutingAlgorithm::shortest)
                          .offset);
      },
      [&](const LanePosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeBoundingBoxRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .offset);
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::shortest, false>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .s);
      },
      [&](const RelativeWorldPosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .s);
      },
      [&](const RelativeObjectPosition & position) {
        return std::abs(makeNativeRelativeLanePosition(
                          triggering_entity, static_cast<NativeLanePosition>(position),
                          RoutingAlgorithm::shortest)
                          .s);
      },
      [&](const LanePosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .s);
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::shortest, true>(
  const EntityRef & triggering_entity, const Position & position) -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeBoundingBoxRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .s);
      },
      [&](const RelativeWorldPosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeBoundingBoxRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .s);
      },
      [&](const RelativeObjectPosition & position) {
        return std::abs(makeNativeBoundingBoxRelativeLanePosition(
                          triggering_entity, static_cast<NativeLanePosition>(position),
                          RoutingAlgorithm::shortest)
                          .s);
      },
      [&](const LanePosition & position) {
        return std::abs(static_cast<traffic_simulator::LaneletPose>(
                          makeNativeBoundingBoxRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest))
                          .s);
      }),
    position);
}

void DistanceCondition::visualize() const
{
  if (relative_distance_type != RelativeDistanceType::euclidianDistance) {
    /// @todo implement visualization for other distance types
    return;
  }
  auto center = static_cast<geometry_msgs::msg::Pose>(position);
  center.orientation.w = 0;

  const auto make_label_marker = [&](auto && triggering_entity) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "distance_condition/" + triggering_entity.name();
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = center;
    marker.pose.position.z += 0.3;
    marker.text = description();
    marker.scale.z = 0.3;
    marker.color.a = 0.8f;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    return marker;
  };

  const auto make_distance_marker = [&](auto && triggering_entity) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "distance_condition/" + triggering_entity.name();
    marker.id = 3;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    auto relative_pose = makeNativeRelativeWorldPosition(center, triggering_entity.name());
    marker.points.push_back(center.position);
    auto entity_position = center.position;
    entity_position.x -= relative_pose.position.x;
    entity_position.y -= relative_pose.position.y;
    entity_position.z -= relative_pose.position.z;
    marker.points.push_back(entity_position);
    marker.scale.x = 0.1;
    marker.color.a = 0.8f;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    return marker;
  };

  const auto make_distance_label_marker = [&](auto && triggering_entity) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "distance_condition/" + triggering_entity.name();
    marker.id = 4;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    auto relative_pose = makeNativeRelativeWorldPosition(center, triggering_entity.name());
    marker.pose.position.x = center.position.x - relative_pose.position.x / 2;
    marker.pose.position.y = center.position.y - relative_pose.position.y / 2;
    marker.pose.position.z = center.position.z - relative_pose.position.z / 2;
    marker.text = std::to_string(distance(triggering_entity));
    marker.scale.z = 0.3;
    marker.color.a = 0.8f;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    return marker;
  };

  std::for_each(
    triggering_entities.entity_refs.begin(), triggering_entities.entity_refs.end(),
    [&](auto && triggering_entity) {
      triggering_entity.apply([&](auto && object) {
        add(make_label_marker(object));
        add(make_distance_marker(object));
        add(make_distance_label_marker(object));
      });
    });
}

auto DistanceCondition::evaluate() -> Object
{
  results.clear();

  auto result = asBoolean(triggering_entities.apply([&](const auto & triggering_entity) {
    results.push_back(triggering_entity.apply([&](const auto & triggering_entity) {
      return evaluate(
        global().entities, triggering_entity, position, coordinate_system, relative_distance_type,
        routing_algorithm, freespace);
    }));
    return not results.back().size() or rule(results.back(), value).min();
  }));

  call_visualize([this]() { visualize(); });

  return result;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
