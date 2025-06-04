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

#ifndef OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_
#define OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_

#include <openscenario_interpreter/cmath/hypot.hpp>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/routing_algorithm.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/unsigned_integer.hpp>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>

namespace openscenario_interpreter
{
using NativeWorldPosition = geometry_msgs::msg::Pose;

using NativeRelativeWorldPosition = NativeWorldPosition;

using NativeLanePosition = traffic_simulator::LaneletPose;

using NativeRelativeLanePosition = NativeLanePosition;

class SimulatorCore
{
  static inline std::unique_ptr<traffic_simulator::API> core = nullptr;

public:
  template <typename NodeType, typename... Ts>
  static auto activate(
    const NodeType & node, const traffic_simulator::Configuration & configuration, Ts &&... xs)
    -> void
  {
    if (not active()) {
      core = std::make_unique<traffic_simulator::API>(
        node, configuration, std::forward<decltype(xs)>(xs)...);
    } else {
      throw Error("The simulator core has already been instantiated.");
    }
  }

  static auto activateNonUserDefinedControllers() -> void { core->startNpcLogic(); }

  static auto active() -> bool { return static_cast<bool>(core); }

  static auto deactivate() -> void
  {
    if (active()) {
      core->despawnEntities();
      core->closeZMQConnection();
      core.reset();
    }
  }

  static auto update() -> void { core->updateFrame(); }

  class CoordinateSystemConversion
  {
  protected:
    /// @brief Returns true if the entity exists or the argument is not an entity name
    template <typename PossibleEntityType>
    static auto doesEntityExistIfIsEntityName(const PossibleEntityType & possible_entity_name)
      -> bool
    {
      if constexpr (std::is_convertible_v<PossibleEntityType, std::string>) {
        return core->isEntityExist(possible_entity_name);
      }
      return true;
    }

    template <typename PossibleFirstEntityType, typename PossibleSecondEntityType>
    static auto doesEntityExistIfIsEntityName(
      const PossibleFirstEntityType & first_possible_entity_name,
      const PossibleSecondEntityType & second_possible_entity_name) -> bool
    {
      return doesEntityExistIfIsEntityName(first_possible_entity_name) and
             doesEntityExistIfIsEntityName(second_possible_entity_name);
    }

    static auto convertToNativeLanePosition(const NativeWorldPosition & map_pose)
      -> NativeLanePosition
    {
      constexpr bool include_crosswalk{false};
      constexpr double matching_distance{1.0};
      if (
        const auto lanelet_pose =
          traffic_simulator::pose::toLaneletPose(map_pose, include_crosswalk, matching_distance)) {
        return lanelet_pose.value();
      } else {
        throw Error(
          "The specified WorldPosition = [", map_pose.position.x, ", ", map_pose.position.y, ", ",
          map_pose.position.z,
          "] could not be approximated to the proper Lane. Perhaps the "
          "WorldPosition points to a location where multiple lanes overlap, and "
          "there are at least two or more candidates for a LanePosition that "
          "can be approximated to that WorldPosition. This issue can be "
          "resolved by strictly specifying the location using LanePosition "
          "instead of WorldPosition. Used: include_crosswalk==",
          include_crosswalk, ", matching_distance==", matching_distance, ".");
      }
    }

    static auto convertToNativeWorldPosition(const NativeLanePosition & lanelet_pose)
      -> NativeWorldPosition
    {
      return traffic_simulator::pose::toMapPose(lanelet_pose);
    }

    template <typename FirstEntityNameOrMapPositionType, typename SecondEntityNameOrMapPositionType>
    static auto makeNativeRelativeWorldPosition(
      const FirstEntityNameOrMapPositionType & from_entity_name_or_map_position,
      const SecondEntityNameOrMapPositionType & to_entity_name_or_map_position)
      -> NativeRelativeWorldPosition
    {
      if (doesEntityExistIfIsEntityName(
            from_entity_name_or_map_position, to_entity_name_or_map_position)) {
        if (
          const auto relative_pose =
            core->relativePose(from_entity_name_or_map_position, to_entity_name_or_map_position)) {
          return relative_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }

    template <
      typename FirstEntityNameOrLanePositionType, typename SecondEntityNameOrLanePositionType>
    static auto makeNativeRelativeLanePosition(
      const FirstEntityNameOrLanePositionType & from_entity_name_or_lane_position,
      const SecondEntityNameOrLanePositionType & to_entity_name_or_lane_position,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> NativeRelativeLanePosition
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);

      if (doesEntityExistIfIsEntityName(
            from_entity_name_or_lane_position, to_entity_name_or_lane_position)) {
        if (
          const auto relative_lanelet_pose = core->relativeLaneletPose(
            from_entity_name_or_lane_position, to_entity_name_or_lane_position,
            routing_configuration)) {
          return relative_lanelet_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNLaneletPose();
    }

    template <typename EntityNameOrLanePositionType>
    static auto makeNativeBoundingBoxRelativeLanePosition(
      const std::string & from_entity_name,
      const EntityNameOrLanePositionType & to_entity_name_or_lane_position,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> NativeRelativeLanePosition
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);

      if (doesEntityExistIfIsEntityName(from_entity_name, to_entity_name_or_lane_position)) {
        if (
          const auto relative_lanelet_pose = core->boundingBoxRelativeLaneletPose(
            from_entity_name, to_entity_name_or_lane_position, routing_configuration)) {
          return relative_lanelet_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNLaneletPose();
    }

    template <typename EntityNameOrMapPositionType>
    static auto makeNativeBoundingBoxRelativeWorldPosition(
      const std::string & from_entity_name,
      const EntityNameOrMapPositionType & to_entity_name_or_map_position)
      -> NativeRelativeWorldPosition
    {
      if (doesEntityExistIfIsEntityName(from_entity_name, to_entity_name_or_map_position)) {
        if (
          const auto relative_pose =
            core->boundingBoxRelativePose(from_entity_name, to_entity_name_or_map_position)) {
          return relative_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }
  };

  class DistanceConditionEvaluation
  {
  private:
    /// @note this function is necessary because on the OpenScenarioInterpreter side
    /// there is sometimes no check if the entity (triggering, reference) exists
    template <typename FirstType, typename SecondType>
    static auto prerequisite(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name) -> bool
    {
      if constexpr (std::is_convertible_v<FirstType, std::string>) {
        if (!core->isEntityExist(from_pose_or_entity_name)) {
          return false;
        }
      }
      if constexpr (std::is_convertible_v<SecondType, std::string>) {
        if (!core->isEntityExist(to_pose_or_entity_name)) {
          return false;
        }
      }
      return true;
    }

  protected:
    // Euclidean distance
    template <typename FirstType, typename SecondType>
    static auto euclideanDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name)
      -> double
    {
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (
          const auto pose = core->relativePose(from_pose_or_entity_name, to_pose_or_entity_name)) {
          return hypot(pose.value().position.x, pose.value().position.y, pose.value().position.z);
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    template <typename FirstType, typename SecondType>
    static auto euclideanBoundingBoxDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name)
      -> double
    {
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (
          const auto pose =
            core->boundingBoxRelativePose(from_pose_or_entity_name, to_pose_or_entity_name)) {
          return hypot(pose.value().position.x, pose.value().position.y, pose.value().position.z);
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    // Entity coordinate system distance
    template <typename FirstType, typename SecondType>
    static auto lateralEntityDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name)
      -> double
    {
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (
          const auto pose = core->relativePose(from_pose_or_entity_name, to_pose_or_entity_name)) {
          return pose.value().position.y;
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    template <typename FirstType, typename SecondType>
    static auto longitudinalEntityDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name)
      -> double
    {
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (
          const auto pose = core->relativePose(from_pose_or_entity_name, to_pose_or_entity_name)) {
          return pose.value().position.x;
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    template <typename FirstType, typename SecondType>
    static auto lateralEntityBoundingBoxDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name)
      -> double
    {
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (
          const auto pose =
            core->boundingBoxRelativePose(from_pose_or_entity_name, to_pose_or_entity_name)) {
          return pose.value().position.y;
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    template <typename FirstType, typename SecondType>
    static auto longitudinalEntityBoundingBoxDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name)
      -> double
    {
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (
          const auto pose =
            core->boundingBoxRelativePose(from_pose_or_entity_name, to_pose_or_entity_name)) {
          return pose.value().position.x;
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    // Lane coordinate system distance
    template <typename FirstType, typename SecondType>
    static auto lateralRelativeLanes(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> int
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (
          const auto lane_changes = core->countLaneChanges(
            from_pose_or_entity_name, to_pose_or_entity_name, routing_configuration)) {
          return lane_changes.value().first - lane_changes.value().second;
        }
      }
      throw common::Error(
        "Failed to evaluate lateral relative lanes between ", from_pose_or_entity_name, " and ",
        to_pose_or_entity_name);
    }

    template <typename FirstType, typename SecondType>
    static auto lateralLaneDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> double
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (const auto lanelet_distance = core->laneletDistance(
              from_pose_or_entity_name, to_pose_or_entity_name, routing_configuration);
            lanelet_distance.lateral) {
          return lanelet_distance.lateral.value();
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    template <typename FirstType, typename SecondType>
    static auto longitudinalLaneDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> double
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (const auto lanelet_distance = core->laneletDistance(
              from_pose_or_entity_name, to_pose_or_entity_name, routing_configuration);
            lanelet_distance.longitudinal) {
          return lanelet_distance.longitudinal.value();
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    template <typename FirstType, typename SecondType>
    static auto lateralLaneBoundingBoxDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> double
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (const auto lanelet_distance = core->boundingBoxLaneletDistance(
              from_pose_or_entity_name, to_pose_or_entity_name, routing_configuration);
            lanelet_distance.lateral) {
          return lanelet_distance.lateral.value();
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    template <typename FirstType, typename SecondType>
    static auto longitudinalLaneBoundingBoxDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> double
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (const auto lanelet_distance = core->boundingBoxLaneletDistance(
              from_pose_or_entity_name, to_pose_or_entity_name, routing_configuration);
            lanelet_distance.longitudinal) {
          return lanelet_distance.longitudinal.value();
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }
  };

  class ActionApplication
  {
  protected:
    template <typename... Ts>
    static auto applyAddEntityAction(Ts &&... xs) -> decltype(auto)
    {
      return core->spawn(std::forward<decltype(xs)>(xs)...);
    }

    /// @note this is called during AddEntityAction
    template <typename PerformanceType, typename PropertiesType>
    static auto activatePerformanceAssertion(
      const std::string & entity_name, const PerformanceType & performance,
      const PropertiesType & properties) -> void
    {
      core->getEntity(entity_name)
        .activateOutOfRangeJob(
          -performance.max_speed, +performance.max_speed, -performance.max_deceleration,
          +performance.max_acceleration,
          properties.template get<Double>("minJerk", Double::lowest()),
          properties.template get<Double>("maxJerk", Double::max()));
    }

    template <typename... Ts>
    static auto applyDeleteEntityAction(Ts &&... xs)
    {
      return core->despawn(std::forward<decltype(xs)>(xs)...);
    }

    template <typename ControllerType>
    static auto applyAssignControllerAction(
      const std::string & entity_name, ControllerType && controller) -> void
    {
      auto & entity = core->getEntity(entity_name);
      entity.setVelocityLimit(controller.properties.template get<Double>(
        "maxSpeed", std::numeric_limits<Double::value_type>::max()));

      entity.setBehaviorParameter([&]() {
        /// The default values written in
        /// https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/traffic_simulator_msgs/msg/DynamicConstraints.msg
        auto behavior_parameter = entity.getBehaviorParameter();
        // clang-format off
        behavior_parameter.see_around =                            not controller.properties.template get<Boolean>("isBlind");
        behavior_parameter.dynamic_constraints.max_acceleration =      controller.properties.template get<Double>("maxAcceleration", 10.0);
        behavior_parameter.dynamic_constraints.max_acceleration_rate = controller.properties.template get<Double>("maxAccelerationRate", 3.0);
        behavior_parameter.dynamic_constraints.max_deceleration =      controller.properties.template get<Double>("maxDeceleration", 10.0);
        behavior_parameter.dynamic_constraints.max_deceleration_rate = controller.properties.template get<Double>("maxDecelerationRate", 3.0);
        behavior_parameter.dynamic_constraints.max_speed =             controller.properties.template get<Double>("maxSpeed", 50.0);
        // clang-format on
        return behavior_parameter;
      }());

      if (controller.isAutoware()) {
        core->attachImuSensor(entity_name, [&]() {
          simulation_api_schema::ImuSensorConfiguration configuration;
          configuration.set_entity(entity_name);
          configuration.set_frame_id("base_link");
          configuration.set_add_gravity(true);
          configuration.set_use_seed(true);
          configuration.set_seed(0);
          configuration.set_noise_standard_deviation_orientation(0.01);
          configuration.set_noise_standard_deviation_twist(0.01);
          configuration.set_noise_standard_deviation_acceleration(0.01);
          return configuration;
        }());

        core->attachLidarSensor([&]() {
          simulation_api_schema::LidarConfiguration configuration;

          auto degree_to_radian = [](auto degree) {
            return degree / 180.0 * boost::math::constants::pi<double>();
          };

          // clang-format off
          configuration.set_architecture_type(common::getParameter<std::string>("architecture_type", "awf/universe/20240605"));
          configuration.set_entity(entity_name);
          configuration.set_horizontal_resolution(degree_to_radian(controller.properties.template get<Double>("pointcloudHorizontalResolution", 1.0)));
          configuration.set_lidar_sensor_delay(controller.properties.template get<Double>("pointcloudPublishingDelay"));
          configuration.set_scan_duration(0.1);
          // clang-format on

          const auto vertical_field_of_view = degree_to_radian(
            controller.properties.template get<Double>("pointcloudVerticalFieldOfView", 30.0));

          const auto channels =
            controller.properties.template get<UnsignedInteger>("pointcloudChannels", 16);

          for (std::size_t i = 0; i < channels; ++i) {
            configuration.add_vertical_angles(
              vertical_field_of_view / 2 - vertical_field_of_view / channels * i);
          }

          return configuration;
        }());

        core->attachDetectionSensor([&]() {
          simulation_api_schema::DetectionSensorConfiguration configuration;
          // clang-format off
          configuration.set_architecture_type(common::getParameter<std::string>("architecture_type", "awf/universe/20240605"));
          configuration.set_entity(entity_name);
          configuration.set_detect_all_objects_in_range(controller.properties.template get<Boolean>("isClairvoyant"));
          configuration.set_object_recognition_delay(controller.properties.template get<Double>("detectedObjectPublishingDelay"));
          configuration.set_pos_noise_stddev(controller.properties.template get<Double>("detectedObjectPositionStandardDeviation"));
          configuration.set_probability_of_lost(controller.properties.template get<Double>("detectedObjectMissingProbability"));
          configuration.set_random_seed(controller.properties.template get<UnsignedInteger>("randomSeed"));
          configuration.set_range(controller.properties.template get<Double>("detectionSensorRange",300.0));
          configuration.set_object_recognition_ground_truth_delay(controller.properties.template get<Double>("detectedObjectGroundTruthPublishingDelay"));
          configuration.set_update_duration(0.1);
          // clang-format on
          return configuration;
        }());

        core->attachOccupancyGridSensor([&]() {
          simulation_api_schema::OccupancyGridSensorConfiguration configuration;
          // clang-format off
          configuration.set_architecture_type(common::getParameter<std::string>("architecture_type", "awf/universe/20240605"));
          configuration.set_entity(entity_name);
          configuration.set_filter_by_range(controller.properties.template get<Boolean>("isClairvoyant"));
          configuration.set_height(200);
          configuration.set_range(300);
          configuration.set_resolution(0.5);
          configuration.set_update_duration(0.1);
          configuration.set_width(200);
          // clang-format on
          return configuration;
        }());

        core->attachPseudoTrafficLightDetector([&]() {
          simulation_api_schema::PseudoTrafficLightDetectorConfiguration configuration;
          configuration.set_architecture_type(
            common::getParameter<std::string>("architecture_type", "awf/universe/20240605"));
          return configuration;
        }());

        auto & ego_entity = core->getEgoEntity(entity_name);

        if (controller.properties.contains("allowGoalModification")) {
          ego_entity.setParameter<bool>(
            "allow_goal_modification",
            controller.properties.template get<Boolean>("allowGoalModification"));
        }

        const auto modules = [&]() {
          auto nonparsed_string = controller.properties.template get<String>(
            "featureIdentifiersRequiringExternalPermissionForAutonomousDecisions");
          nonparsed_string.erase(
            std::remove_if(
              nonparsed_string.begin(), nonparsed_string.end(),
              [](const auto & c) { return std::isspace(c); }),
            nonparsed_string.end());

          std::vector<std::string> parsed_string;
          std::string buffer;
          std::istringstream modules_stream(nonparsed_string);
          while (std::getline(modules_stream, buffer, ',')) {
            parsed_string.push_back(buffer);
          }
          return parsed_string;
        }();

        for (const auto & module : modules) {
          try {
            ego_entity.requestAutoModeForCooperation(module, false);
          } catch (const Error & error) {
            throw Error(
              "featureIdentifiersRequiringExternalPermissionForAutonomousDecisions: module ",
              std::quoted(module), " is not supported in this environment, error: ", error.what());
          }
        }
      }
    }

    template <
      typename PoseType, typename... Ts,
      typename = std::enable_if_t<std::disjunction_v<
        std::is_same<std::decay_t<PoseType>, NativeWorldPosition>,
        std::is_same<std::decay_t<PoseType>, NativeRelativeWorldPosition>,
        std::is_same<std::decay_t<PoseType>, NativeLanePosition>,
        std::is_same<std::decay_t<PoseType>, NativeRelativeLanePosition>>>>
    static auto applyTeleportAction(
      const std::string & entity_name, const PoseType & pose, Ts &&... xs)
    {
      return core->getEntity(entity_name).setStatus(pose, std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyTeleportAction(
      const std::string & entity_name, const std::string & reference_entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name)
        .setStatus(
          core->getEntity(reference_entity_name).getMapPose(), std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyAcquirePositionAction(const std::string & entity_name, Ts &&... xs) -> void
    {
      auto & entity = core->getEntity(entity_name);
      entity.requestClearRoute();
      return entity.requestAcquirePosition(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyAssignRouteAction(const std::string & entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name).requestAssignRoute(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyWalkStraightAction(const std::string & entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name).requestWalkStraight(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyFollowTrajectoryAction(const std::string & entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name)
        .requestFollowTrajectory(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyLaneChangeAction(const std::string & entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name).requestLaneChange(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applySpeedAction(const std::string & entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name).requestSpeedChange(std::forward<decltype(xs)>(xs)...);
    }

    template <typename DynamicConstraints>
    static auto applyProfileAction(
      const std::string & entity_name, const DynamicConstraints & dynamic_constraints) -> void
    {
      auto & entity = core->getEntity(entity_name);
      entity.setBehaviorParameter([&entity, &dynamic_constraints] {
        auto behavior_parameter = entity.getBehaviorParameter();

        const auto setIfNotInf = [](auto & target, const auto & value) {
          if (not std::isinf(value)) {
            target = value;
          }
        };
        // clang-format off
        setIfNotInf(behavior_parameter.dynamic_constraints.max_speed,             dynamic_constraints.max_speed);
        setIfNotInf(behavior_parameter.dynamic_constraints.max_acceleration,      dynamic_constraints.max_acceleration);
        setIfNotInf(behavior_parameter.dynamic_constraints.max_acceleration_rate, dynamic_constraints.max_acceleration_rate);
        setIfNotInf(behavior_parameter.dynamic_constraints.max_deceleration,      dynamic_constraints.max_deceleration);
        setIfNotInf(behavior_parameter.dynamic_constraints.max_deceleration_rate, dynamic_constraints.max_deceleration_rate);
        // clang-format on

        return behavior_parameter;
      }());
    }
  };

  class ConditionEvaluation
  {
  protected:
    template <typename... Ts>
    static auto evaluateSimulationTime(Ts &&... xs) -> double
    {
      if (SimulatorCore::active()) {
        return core->getCurrentTime(std::forward<delctype(xs)>(xs)...);
      } else {
        return std::numeric_limits<double>::quiet_NaN();
      }
    }

    static auto evaluateStandStill(const std::string & entity_name) -> double
    {
      if (core->isEntityExist(entity_name)) {
        return core->getEntity(entity_name).getStandStillDuration();
      } else {
        return std::numeric_limits<double>::quiet_NaN();
      }
    }

    static auto evaluateSpeed(const std::string & entity_name) -> Eigen::Vector3d
    {
      if (core->isEntityExist(entity_name)) {
        const auto linear = core->getEntity(entity_name).getCurrentTwist().linear;
        return Eigen::Vector3d(linear.x, linear.y, linear.z);
      } else {
        const auto nan = std::numeric_limits<double>::quiet_NaN();
        return Eigen::Vector3d(nan, nan, nan);
      }
    }

    static auto evaluateRelativeSpeed(
      const std::string & from_entity_name, const std::string & to_entity_name) -> Eigen::Vector3d
    {
      if (core->isEntityExist(from_entity_name) && core->isEntityExist(to_entity_name)) {
        return core->relativeSpeed(from_entity_name, to_entity_name);
      } else {
        return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
      }
    }

    static auto evaluateAcceleration(const std::string & entity_name) -> double
    {
      if (core->isEntityExist(entity_name)) {
        return core->getEntity(entity_name).getCurrentAccel().linear.x;
      } else {
        return std::numeric_limits<double>::quiet_NaN();
      }
    }

    template <typename... Ts>
    static auto evaluateCollisionCondition(
      const std::string & first_entity_name, const std::string & second_entity_name, Ts &&... xs)
      -> bool
    {
      if (core->isEntityExist(first_entity_name) && core->isEntityExist(second_entity_name)) {
        return core->checkCollision(
          first_entity_name, second_entity_name, std::forward<decltype(xs)>(xs)...);
      } else {
        return false;
      }
    }

    static auto evaluateTimeHeadway(
      const std::string & from_entity_name, const std::string & to_entity_name) -> double
    {
      if (core->isEntityExist(from_entity_name) && core->isEntityExist(to_entity_name)) {
        if (const auto time_headway = core->timeHeadway(from_entity_name, to_entity_name)) {
          return time_headway.value();
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    // User defined conditions
    template <typename... Ts>
    static auto evaluateCurrentState(const std::string & entity_name, Ts &&... xs) -> std::string
    {
      if (core->isEntityExist(entity_name)) {
        return core->getEntity(entity_name).getCurrentAction(std::forward<decltype(xs)>(xs)...);
      } else {
        return "not spawned";
      }
    }

    template <typename OSCLanePosition>
    static auto evaluateRelativeHeading(
      const std::string & entity_name, const OSCLanePosition & osc_lane_position)
    {
      if (core->isEntityExist(entity_name)) {
        const auto lane_pose = static_cast<NativeLanePosition>(osc_lane_position);
        if (const auto relative_yaw = core->laneletRelativeYaw(entity_name, lane_pose)) {
          return static_cast<Double>(std::abs(relative_yaw.value()));
        }
      }
      return Double::nan();
    }

    static auto evaluateRelativeHeading(const std::string & entity_name)
    {
      if (core->isEntityExist(entity_name)) {
        if (const auto relative_yaw = core->getEntity(entity_name).getLaneletRelativeYaw()) {
          return static_cast<Double>(std::abs(relative_yaw.value()));
        }
      }
      return Double::nan();
    }
  };

  class NonStandardOperation
  {
  protected:
    static auto engage(const std::string & ego_name) -> void
    {
      core->getEgoEntity(ego_name).engage();
    }

    static auto isEngageable(const std::string & ego_name) -> bool
    {
      return core->getEgoEntity(ego_name).isEngageable();
    }

    static auto isEngaged(const std::string & ego_name) -> bool
    {
      return core->getEgoEntity(ego_name).isEngaged();
    }

    template <typename... Ts>
    static auto sendCooperateCommand(Ts &&... xs) -> decltype(auto)
    {
      /// @note here ego name is not passed from OpenScenarioInterpreter, it uses first found
      if (const auto ego_name = core->getFirstEgoName()) {
        core->getEgoEntity(ego_name.value())
          .sendCooperateCommand(std::forward<decltype(xs)>(xs)...);
      } else {
        throw common::Error("No ego entity exists.");
      }
    }

    static auto getMinimumRiskManeuverBehaviorName(const std::string & ego_name) -> std::string
    {
      return core->getEgoEntity(ego_name).getMinimumRiskManeuverBehaviorName();
    }

    static auto getMinimumRiskManeuverStateName(const std::string & ego_name) -> std::string
    {
      return core->getEgoEntity(ego_name).getMinimumRiskManeuverStateName();
    }

    static auto getEmergencyStateName(const std::string & ego_name) -> std::string
    {
      return core->getEgoEntity(ego_name).getEmergencyStateName();
    }

    static auto getTurnIndicatorsCommandName(const std::string & ego_name) -> std::string
    {
      return core->getEgoEntity(ego_name).getTurnIndicatorsCommandName();
    }
  };

  class TrafficLightsOperation
  {
  protected:
    template <typename... Ts>
    static auto setConventionalTrafficLightsState(Ts &&... xs) -> decltype(auto)
    {
      return core->getConventionalTrafficLights()->setTrafficLightsState(
        std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto setConventionalTrafficLightConfidence(Ts &&... xs) -> decltype(auto)
    {
      return core->getConventionalTrafficLights()->setTrafficLightsConfidence(
        std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto getConventionalTrafficLightsComposedState(Ts &&... xs) -> decltype(auto)
    {
      return core->getConventionalTrafficLights()->getTrafficLightsComposedState(
        std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto compareConventionalTrafficLightsState(Ts &&... xs) -> decltype(auto)
    {
      return core->getConventionalTrafficLights()->compareTrafficLightsState(
        std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto resetConventionalTrafficLightPublishRate(Ts &&... xs) -> decltype(auto)
    {
      return core->getConventionalTrafficLights()->resetUpdate(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto setV2ITrafficLightsState(Ts &&... xs) -> decltype(auto)
    {
      return core->getV2ITrafficLights()->setTrafficLightsState(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto resetV2ITrafficLightPublishRate(Ts &&... xs) -> decltype(auto)
    {
      return core->getV2ITrafficLights()->resetUpdate(std::forward<decltype(xs)>(xs)...);
    }
  };
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_
