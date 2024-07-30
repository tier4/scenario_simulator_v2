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

#include <cmath>
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
  results(triggering_entities.entity_refs.size(), {Double::nan()}),
  consider_z([]() {
    rclcpp::Node node{"get_parameter", "simulation"};
    node.declare_parameter("consider_pose_by_road_slope", false);
    return node.get_parameter("consider_pose_by_road_slope").as_bool();
  }())
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

#define SWITCH_ROUTING_ALGORITHM(FUNCTION, ...)                     \
  switch (routing_algorithm) {                                      \
    case RoutingAlgorithm::assigned_route:                          \
      FUNCTION(__VA_ARGS__, RoutingAlgorithm::assigned_route);      \
      break;                                                        \
    case RoutingAlgorithm::fastest:                                 \
      FUNCTION(__VA_ARGS__, RoutingAlgorithm::fastest);             \
      break;                                                        \
    case RoutingAlgorithm::least_intersections:                     \
      FUNCTION(__VA_ARGS__, RoutingAlgorithm::least_intersections); \
      break;                                                        \
    case RoutingAlgorithm::shortest:                                \
      FUNCTION(__VA_ARGS__, RoutingAlgorithm::shortest);            \
      break;                                                        \
    case RoutingAlgorithm::undefined:                               \
      FUNCTION(__VA_ARGS__, RoutingAlgorithm::undefined);           \
      break;                                                        \
  }

#define SWITCH_FREESPACE(FUNCTION, ...) \
  return freespace ? FUNCTION(__VA_ARGS__, true) : FUNCTION(__VA_ARGS__, false)

#define DISTANCE(...) distance<__VA_ARGS__>(triggering_entity)

auto DistanceCondition::distance(const EntityRef & triggering_entity) const -> double
{
  SWITCH_COORDINATE_SYSTEM(
    SWITCH_RELATIVE_DISTANCE_TYPE, SWITCH_ROUTING_ALGORITHM, SWITCH_FREESPACE, DISTANCE);
  return Double::nan();
}

// @todo: after checking all the scenario work well with consider_z = true, remove this function and use std::hypot(x,y,z)
static double hypot(const double x, const double y, const double z, const bool consider_z)
{
  return consider_z ? std::hypot(x, y, z) : std::hypot(x, y);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, RoutingAlgorithm::undefined,
  false>(const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        const auto relative_world = makeNativeRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z,
          consider_z);
      },
      [&](const RelativeWorldPosition & position) {
        const auto relative_world = makeNativeRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z,
          consider_z);
      },
      [&](const RelativeObjectPosition & position) {
        const auto relative_world = makeNativeRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z,
          consider_z);
      },
      [&](const LanePosition & position) {
        const auto relative_world = makeNativeRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z,
          consider_z);
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, RoutingAlgorithm::undefined,
  true>(const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        const auto relative_world = makeNativeBoundingBoxRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z,
          consider_z);
      },
      [&](const RelativeWorldPosition & position) {
        const auto relative_world = makeNativeBoundingBoxRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z,
          consider_z);
      },
      [&](const RelativeObjectPosition & position) {
        const auto relative_world = makeNativeBoundingBoxRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z,
          consider_z);
      },
      [&](const LanePosition & position) {
        const auto relative_world = makeNativeBoundingBoxRelativeWorldPosition(
          triggering_entity, static_cast<NativeWorldPosition>(position));
        return hypot(
          relative_world.position.x, relative_world.position.y, relative_world.position.z,
          consider_z);
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::entity, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity) const -> double
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
  const EntityRef & triggering_entity) const -> double
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
  const EntityRef & triggering_entity) const -> double
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
  const EntityRef & triggering_entity) const -> double
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
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .offset;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeWorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .offset;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeObjectPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return makeNativeRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position))
            .offset;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const LanePosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .offset;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeBoundingBoxRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .offset;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeWorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeBoundingBoxRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .offset;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeObjectPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return makeNativeBoundingBoxRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position))
            .offset;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const LanePosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeBoundingBoxRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .offset;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, false>(
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .s;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeWorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .s;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeObjectPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return makeNativeRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position))
            .s;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const LanePosition & position) {
        if (global().entities->ref(triggering_entity).template as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .s;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::undefined, true>(
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeBoundingBoxRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .s;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeWorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeBoundingBoxRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .s;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeObjectPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return makeNativeBoundingBoxRelativeLanePosition(
                   triggering_entity, static_cast<NativeLanePosition>(position))
            .s;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const LanePosition & position) {
        if (global().entities->ref(triggering_entity).template as<ScenarioObject>().is_added) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeBoundingBoxRelativeLanePosition(
                     triggering_entity, static_cast<NativeLanePosition>(position)))
            .s;
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::shortest, false>(
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .offset);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeWorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .offset);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeObjectPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(makeNativeRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest)
                            .offset);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const LanePosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .offset);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::lateral, RoutingAlgorithm::shortest, true>(
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeBoundingBoxRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .offset);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeWorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeBoundingBoxRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .offset);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeObjectPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(makeNativeBoundingBoxRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest)
                            .offset);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const LanePosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeBoundingBoxRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .offset);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::shortest, false>(
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .s);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeWorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .s);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeObjectPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(makeNativeRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest)
                            .s);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const LanePosition & position) {
        if (global().entities->ref(triggering_entity).template as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .s);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      }),
    position);
}

template <>
auto DistanceCondition::distance<
  CoordinateSystem::lane, RelativeDistanceType::longitudinal, RoutingAlgorithm::shortest, true>(
  const EntityRef & triggering_entity) const -> double
{
  return apply<double>(
    overload(
      [&](const WorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeBoundingBoxRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .s);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeWorldPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeBoundingBoxRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .s);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const RelativeObjectPosition & position) {
        if (global().entities->ref(triggering_entity).as<ScenarioObject>().is_added) {
          return std::abs(makeNativeBoundingBoxRelativeLanePosition(
                            triggering_entity, static_cast<NativeLanePosition>(position),
                            RoutingAlgorithm::shortest)
                            .s);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
      },
      [&](const LanePosition & position) {
        if (global().entities->ref(triggering_entity).template as<ScenarioObject>().is_added) {
          return std::abs(static_cast<traffic_simulator::LaneletPose>(
                            makeNativeBoundingBoxRelativeLanePosition(
                              triggering_entity, static_cast<NativeLanePosition>(position),
                              RoutingAlgorithm::shortest))
                            .s);
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
    results.push_back(
      triggering_entity.apply([&](const auto & object) { return distance(object); }));
    return not results.back().size() or rule(results.back(), value).min();
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
