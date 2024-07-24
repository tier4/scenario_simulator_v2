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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
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

using NativeLanePosition = traffic_simulator::CanonicalizedLaneletPose;

using NativeRelativeLanePosition = traffic_simulator::LaneletPose;

class SimulatorCore
{
  static inline std::unique_ptr<traffic_simulator::API> core = nullptr;

public:
  template <typename Node, typename... Ts>
  static auto activate(
    const Node & node, const traffic_simulator::Configuration & configuration, Ts &&... xs) -> void
  {
    if (not active()) {
      core = std::make_unique<traffic_simulator::API>(
        node, configuration, std::forward<decltype(xs)>(xs)...);
    } else {
      throw Error("The simulator core has already been instantiated.");
    }
  }

  static auto active() { return static_cast<bool>(core); }

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
    static auto canonicalize(const traffic_simulator::LaneletPose & non_canonicalized)
      -> NativeLanePosition
    {
      return NativeLanePosition(non_canonicalized, core->getHdmapUtils());
    }

    template <typename T, typename std::enable_if_t<std::is_same_v<T, NativeLanePosition>, int> = 0>
    static auto convert(const NativeWorldPosition & pose) -> NativeLanePosition
    {
      constexpr bool include_crosswalk{false};
      if (
        const auto result = traffic_simulator::pose::toCanonicalizedLaneletPose(
          pose, include_crosswalk, core->getHdmapUtils())) {
        return result.value();
      } else {
        throw Error(
          "The specified WorldPosition = [", pose.position.x, ", ", pose.position.y, ", ",
          pose.position.z,
          "] could not be approximated to the proper Lane. Perhaps the "
          "WorldPosition points to a location where multiple lanes overlap, and "
          "there are at least two or more candidates for a LanePosition that "
          "can be approximated to that WorldPosition. This issue can be "
          "resolved by strictly specifying the location using LanePosition "
          "instead of WorldPosition");
      }
    }

    template <
      typename T, typename std::enable_if_t<std::is_same_v<T, NativeWorldPosition>, int> = 0>
    static auto convert(const NativeLanePosition & native_lane_position) -> NativeWorldPosition
    {
      return traffic_simulator::pose::toMapPose(native_lane_position);
    }

    template <typename FirstType, typename SecondType>
    static auto prerequisite(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name) -> bool
    {
      if constexpr (std::is_same_v<FirstType, std::string>) {
        if (!core->isEntitySpawned(from_pose_or_entity_name)) {
          return false;
        }
      }
      if constexpr (std::is_same_v<SecondType, std::string>) {
        if (!core->isEntitySpawned(to_pose_or_entity_name)) {
          return false;
        }
      }
      return true;
    }

    // Euclidean distance
    template <typename FirstType, typename SecondType>
    static auto euclideanDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name,
      const bool consider_z) -> double
    {
      const auto hypot = [&](const double x, const double y, const double z) {
        return consider_z ? std::hypot(x, y, z) : std::hypot(x, y);
      };

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
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name,
      const bool consider_z) -> double
    {
      const auto hypot = [&](const double x, const double y, const double z) {
        return consider_z ? std::hypot(x, y, z) : std::hypot(x, y);
      };

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
    static auto lateralLaneDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> double
    {
      const bool allow_lane_change = (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (
          const auto pose = core->relativeLaneletPose(
            from_pose_or_entity_name, to_pose_or_entity_name, allow_lane_change)) {
          return pose.value().offset;
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    template <typename FirstType, typename SecondType>
    static auto longitudinalLaneDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> double
    {
      const bool allow_lane_change = (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (
          const auto pose = core->relativeLaneletPose(
            from_pose_or_entity_name, to_pose_or_entity_name, allow_lane_change)) {
          return pose.value().s;
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    template <typename FirstType, typename SecondType>
    static auto lateralLaneBoundingBoxDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> double
    {
      const bool allow_lane_change = (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (
          const auto pose = core->boundingBoxRelativeLaneletPose(
            from_pose_or_entity_name, to_pose_or_entity_name, allow_lane_change)) {
          return pose.value().offset;
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    template <typename FirstType, typename SecondType>
    static auto longitudinalLaneBoundingBoxDistance(
      const FirstType & from_pose_or_entity_name, const SecondType & to_pose_or_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> double
    {
      const bool allow_lane_change = (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      if (prerequisite(from_pose_or_entity_name, to_pose_or_entity_name)) {
        if (
          const auto pose = core->boundingBoxRelativeLaneletPose(
            from_pose_or_entity_name, to_pose_or_entity_name, allow_lane_change)) {
          return pose.value().s;
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    // other necessary now
    static auto makeNativeRelativeWorldPosition(
      const NativeWorldPosition & from_map_pose, const std::string & to_entity_name)
    {
      if (core->isEntitySpawned(to_entity_name)) {
        if (const auto relative_pose = core->relativePose(from_map_pose, to_entity_name)) {
          return relative_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }
  };

  class ActionApplication  // OpenSCENARIO 1.1.1 Section 3.1.5
  {
  protected:
    template <typename... Ts>
    static auto applyAcquirePositionAction(const std::string & entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name)->requestAcquirePosition(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyAddEntityAction(const std::string & entity_name, Ts &&... xs)
    {
      return core->spawn(entity_name, std::forward<decltype(xs)>(xs)...);
    }

    template <typename DynamicConstraints>
    static auto applyProfileAction(
      const std::string & entity_name, const DynamicConstraints & dynamic_constraints) -> void
    {
      auto entity = core->getEntity(entity_name);
      return entity->setBehaviorParameter([&]() {
        auto behavior_parameter = entity->getBehaviorParameter();

        if (not std::isinf(dynamic_constraints.max_speed)) {
          behavior_parameter.dynamic_constraints.max_speed = dynamic_constraints.max_speed;
        }

        if (not std::isinf(dynamic_constraints.max_acceleration)) {
          behavior_parameter.dynamic_constraints.max_acceleration =
            dynamic_constraints.max_acceleration;
        }

        if (not std::isinf(dynamic_constraints.max_acceleration_rate)) {
          behavior_parameter.dynamic_constraints.max_acceleration_rate =
            dynamic_constraints.max_acceleration_rate;
        }

        if (not std::isinf(dynamic_constraints.max_deceleration)) {
          behavior_parameter.dynamic_constraints.max_deceleration =
            dynamic_constraints.max_deceleration;
        }

        if (not std::isinf(dynamic_constraints.max_deceleration_rate)) {
          behavior_parameter.dynamic_constraints.max_deceleration_rate =
            dynamic_constraints.max_deceleration_rate;
        }

        return behavior_parameter;
      }());
    }

    template <typename Controller>
    static auto applyAssignControllerAction(
      const std::string & entity_name, Controller && controller) -> void
    {
      auto entity = core->getEntity(entity_name);
      entity->setVelocityLimit(controller.properties.template get<Double>(
        "maxSpeed", std::numeric_limits<Double::value_type>::max()));

      entity->setBehaviorParameter([&]() {
        auto message = entity->getBehaviorParameter();
        message.see_around = not controller.properties.template get<Boolean>("isBlind");
        /// The default values written in https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/traffic_simulator_msgs/msg/DynamicConstraints.msg
        message.dynamic_constraints.max_acceleration =
          controller.properties.template get<Double>("maxAcceleration", 10.0);
        message.dynamic_constraints.max_acceleration_rate =
          controller.properties.template get<Double>("maxAccelerationRate", 3.0);
        message.dynamic_constraints.max_deceleration =
          controller.properties.template get<Double>("maxDeceleration", 10.0);
        message.dynamic_constraints.max_deceleration_rate =
          controller.properties.template get<Double>("maxDecelerationRate", 3.0);
        message.dynamic_constraints.max_speed =
          controller.properties.template get<Double>("maxSpeed", 50.0);
        return message;
      }());

      if (controller.isAutoware()) {
        auto ego_entity = core->getEgoEntity(entity_name);
        core->attachLidarSensor(
          entity_name, controller.properties.template get<Double>("pointcloudPublishingDelay"));

        core->attachDetectionSensor([&]() {
          simulation_api_schema::DetectionSensorConfiguration configuration;
          // clang-format off
          configuration.set_architecture_type(core->getROS2Parameter<std::string>("architecture_type", "awf/universe"));
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
          configuration.set_architecture_type(core->getROS2Parameter<std::string>("architecture_type", "awf/universe"));
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
            core->getROS2Parameter<std::string>("architecture_type", "awf/universe"));
          return configuration;
        }());

        ego_entity->setParameter<bool>(
          "allow_goal_modification",
          controller.properties.template get<Boolean>("allowGoalModification"));

        for (const auto & module :
             [](std::string manual_modules_string) {
               manual_modules_string.erase(
                 std::remove_if(
                   manual_modules_string.begin(), manual_modules_string.end(),
                   [](const auto & c) { return std::isspace(c); }),
                 manual_modules_string.end());

               std::vector<std::string> modules;
               std::string buffer;
               std::istringstream modules_stream(manual_modules_string);
               while (std::getline(modules_stream, buffer, ',')) {
                 modules.push_back(buffer);
               }
               return modules;
             }(controller.properties.template get<String>(
               "featureIdentifiersRequiringExternalPermissionForAutonomousDecisions"))) {
          try {
            ego_entity->requestAutoModeForCooperation(module, false);
          } catch (const Error & error) {
            throw Error(
              "featureIdentifiersRequiringExternalPermissionForAutonomousDecisions is not "
              "supported in this environment: ",
              error.what());
          }
        }
      }
    }

    static auto applyAssignRouteAction(
      const std::string & entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name)->requestAssignRoute(std::forward<decltype(xs)>(xs)...);
    }

    static auto applyDeleteEntityAction(const std::string & entity_name)
    {
      return core->despawn(entity_name);
    }

    static auto applyFollowTrajectoryAction(
      const std::string & entity_name,
      const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & parameter)
    {
      return core->getEntity(entity_name)->requestFollowTrajectory(parameter);
    }

    template <typename... Ts>
    static auto applyLaneChangeAction(const std::string & entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name)->requestLaneChange(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applySpeedAction(const std::string & entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name)->requestSpeedChange(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyTeleportAction(const std::string & entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name)->setStatus(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyTeleportAction(
      const std::string & entity_name, const std::string & reference_entity_name, Ts &&... xs)
    {
      return core->getEntity(entity_name)
        ->setStatus(
          core->getEntity(reference_entity_name)->getMapPose(), std::forward<decltype(xs)>(xs)...);
    }

    static auto applyWalkStraightAction(const std::string & entity_name)
    {
      return core->getEntity(entity_name)->requestWalkStraight();
    }
  };

  // OpenSCENARIO 1.1.1 Section 3.1.5
  class ConditionEvaluation : protected CoordinateSystemConversion
  {
  protected:
    static auto evaluateAcceleration(const std::string & entity_name)
    {
      return core->getEntity(entity_name)->getCurrentAccel().linear.x;
    }

    static auto evaluateCollisionCondition(
      const std::string & first_entity_name, const std::string & second_entity_name)
    {
      if (prerequisite(first_entity_name, second_entity_name)) {
        return core->checkCollision(first_entity_name, second_entity_name);
      }
      return false;
    }

    static auto evaluateBoundingBoxEuclideanDistance(
      const std::string & from_entity_name, const std::string & to_entity_name)
    {
      if (prerequisite(from_entity_name, to_entity_name)) {
        if (const auto distance = core->boundingBoxDistance(from_entity_name, to_entity_name)) {
          return distance.value();
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    static auto evaluateSimulationTime()
    {
      if (SimulatorCore::active()) {
        return core->getCurrentTime();
      } else {
        return std::numeric_limits<double>::quiet_NaN();
      }
    }

    static auto evaluateSpeed(const std::string & entity_name)
    {
      return core->getEntity(entity_name)->getCurrentTwist().linear.x;
    }

    static auto evaluateStandStill(const std::string & entity_name)
    {
      return core->getEntity(entity_name)->getStandStillDuration();
    }

    static auto evaluateTimeHeadway(
      const std::string & from_entity_name, const std::string & to_entity_name)
    {
      if (prerequisite(from_entity_name, to_entity_name)) {
        if (const auto time_headway = core->timeHeadway(from_entity_name, to_entity_name)) {
          return time_headway.value();
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }
  };

  class NonStandardOperation : private CoordinateSystemConversion
  {
  protected:
    template <typename Performance, typename Properties>
    static auto activatePerformanceAssertion(
      const std::string & entity_name, const Performance & performance,
      const Properties & properties)
    {
      core->getEntity(entity_name)
        ->activateOutOfRangeJob(
          -performance.max_speed, +performance.max_speed, -performance.max_deceleration,
          +performance.max_acceleration,
          properties.template get<Double>("minJerk", Double::lowest()),
          properties.template get<Double>("maxJerk", Double::max()));
    }

    static auto activateNonUserDefinedControllers() { return core->startNpcLogic(); }

    // Evaluate - user defined condition
    static auto evaluateCurrentState(const std::string & entity_name)
    {
      return core->getEntity(entity_name)->getCurrentAction();
    }

    static auto evaluateRelativeHeading(
      const std::string & entity_name, const traffic_simulator::CanonicalizedLaneletPose & lanelet_pose)
    {
      if (core->isEntitySpawned(entity_name)) {
        if (const auto relative_yaw = core->laneletRelativeYaw(entity_name, lanelet_pose)) {
          return std::abs(relative_yaw.value());
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    static auto evaluateRelativeHeading(const std::string & entity_name)
    {
      if (core->isEntitySpawned(entity_name)) {
        if (const auto relative_yaw = core->getEntity(entity_name)->getLaneletRelativeYaw()) {
          return std::abs(relative_yaw.value());
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    // Ego
    static auto engage(const std::string & ego_name)
    {
      return core->getEgoEntity(ego_name)->engage();
    }

    static auto isEngageable(const std::string & ego_name)
    {
      return core->getEgoEntity(ego_name)->isEngageable();
    }

    static auto isEngaged(const std::string & ego_name)
    {
      return core->getEgoEntity(ego_name)->isEngaged();
    }

    static auto sendCooperateCommand(const std::string & module_name, const std::string & command)
    {
      /// @note here ego name is not passed from OpenScenarioInterpreter, it uses first found
      return core->getEgoEntity()->sendCooperateCommand(module_name, command);
    }

    static auto getMinimumRiskManeuverBehaviorName(const std::string & ego_name)
    {
      return core->getEgoEntity(ego_name)->getMinimumRiskManeuverBehaviorName();
    }

    static auto getMinimumRiskManeuverStateName(const std::string & ego_name)
    {
      return core->getEgoEntity(ego_name)->getMinimumRiskManeuverStateName();
    }

    static auto getEmergencyStateName(const std::string & ego_name)
    {
      return core->getEgoEntity(ego_name)->getEmergencyStateName();
    }

    static auto getTurnIndicatorsCommandName(const std::string & ego_name)
    {
      return core->getEgoEntity(ego_name)->getTurnIndicatorsCommandName();
    }

    // TrafficLights - Conventional and V2I
    static auto setConventionalTrafficLightsState(
      const lanelet::Id lanelet_id, const std::string & state)
    {
      return core->getConventionalTrafficLights()->setTrafficLightsState(lanelet_id, state);
    }

    static auto setConventionalTrafficLightConfidence(
      const lanelet::Id lanelet_id, const double confidence)
    {
      return core->getConventionalTrafficLights()->setTrafficLightsConfidence(
        lanelet_id, confidence);
    }

    static auto getConventionalTrafficLightsComposedState(const lanelet::Id lanelet_id)
    {
      return core->getConventionalTrafficLights()->getTrafficLightsComposedState(lanelet_id);
    }

    static auto compareConventionalTrafficLightsState(
      const lanelet::Id lanelet_id, const std::string & states)
    {
      return core->getConventionalTrafficLights()->compareTrafficLightsState(lanelet_id, states);
    }

    static auto resetConventionalTrafficLightPublishRate(const double update_rate)
    {
      return core->getConventionalTrafficLights()->resetUpdate(update_rate);
    }

    static auto setV2ITrafficLightsState(const lanelet::Id lanelet_id, const std::string & state)
    {
      return core->getV2ITrafficLights()->setTrafficLightsState(lanelet_id, state);
    }

    static auto resetV2ITrafficLightPublishRate(const double update_rate)
    {
      return core->getV2ITrafficLights()->resetUpdate(update_rate);
    }
  };
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_
