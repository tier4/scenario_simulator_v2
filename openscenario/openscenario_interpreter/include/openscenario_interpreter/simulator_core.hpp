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

#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
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
      return NativeLanePosition(non_canonicalized);
    }

    template <typename T, typename std::enable_if_t<std::is_same_v<T, NativeLanePosition>, int> = 0>
    static auto convert(const NativeWorldPosition & pose) -> NativeLanePosition
    {
      constexpr bool include_crosswalk{false};
      if (const auto result =
            traffic_simulator::pose::toCanonicalizedLaneletPoses(pose, include_crosswalk);
          !result.empty()) {
        // WIP just use first lanelet pose, should be changed in the future
        return result.front();
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

    static auto makeNativeRelativeWorldPosition(
      const std::string & from_entity_name, const std::string & to_entity_name)
    {
      if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
        if (const auto to_entity = core->getEntityPointer(to_entity_name)) {
          if (
            const auto relative_pose = traffic_simulator::pose::relativePose(
              from_entity->getMapPose(), to_entity->getMapPose()))
            return relative_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }

    static auto makeNativeRelativeWorldPosition(
      const std::string & from_entity_name, const NativeWorldPosition & to_map_pose)
    {
      if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
        if (
          const auto relative_pose =
            traffic_simulator::pose::relativePose(from_entity->getMapPose(), to_map_pose)) {
          return relative_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }

    static auto makeNativeRelativeWorldPosition(
      const NativeWorldPosition & from_map_pose, const std::string & to_entity_name)
    {
      if (const auto to_entity = core->getEntityPointer(to_entity_name)) {
        if (
          const auto relative_pose =
            traffic_simulator::pose::relativePose(from_map_pose, to_entity->getMapPose())) {
          return relative_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }

    static auto makeNativeRelativeLanePosition(
      const std::string & from_entity_name, const std::string & to_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose
    {
      if (const auto to_entity = core->getEntityPointer(to_entity_name)) {
        if (const auto to_lanelet_poses = to_entity->getCanonicalizedLaneletPoses();
            !to_lanelet_poses.empty()) {
          return makeNativeRelativeLanePosition(
            // WIP just use first lanelet pose, should be changed in the future
            from_entity_name, to_lanelet_poses.front(), routing_algorithm);
        }
      }
      return traffic_simulator::pose::quietNaNLaneletPose();
    }

    static auto makeNativeRelativeLanePosition(
      const std::string & from_entity_name, const NativeLanePosition & to_lanelet_pose,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose
    {
      if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
        if (const auto from_lanelet_poses = from_entity->getCanonicalizedLaneletPoses();
            !from_lanelet_poses.empty()) {
          return makeNativeRelativeLanePosition(
            from_lanelet_poses.front(), to_lanelet_pose, routing_algorithm);
        }
      }
      return traffic_simulator::pose::quietNaNLaneletPose();
    }

    static auto makeNativeRelativeLanePosition(
      const NativeLanePosition & from_lanelet_pose, const NativeLanePosition & to_lanelet_pose,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      return traffic_simulator::pose::relativeLaneletPose(
        from_lanelet_pose, to_lanelet_pose, routing_configuration);
    }

    static auto makeNativeBoundingBoxRelativeLanePosition(
      const std::string & from_entity_name, const std::string & to_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
    {
      if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
        if (const auto to_entity = core->getEntityPointer(to_entity_name)) {
          if (const auto from_lanelet_poses = from_entity->getCanonicalizedLaneletPoses();
              !from_lanelet_poses.empty()) {
            if (const auto to_lanelet_poses = to_entity->getCanonicalizedLaneletPoses();
                !to_lanelet_poses.empty()) {
              return makeNativeBoundingBoxRelativeLanePosition(
                // WIP just use first lanelet pose, should be changed in the future
                from_lanelet_poses.front(), from_entity->getBoundingBox(), to_lanelet_poses.front(),
                to_entity->getBoundingBox(), routing_algorithm);
            }
          }
        }
      }
      return traffic_simulator::pose::quietNaNLaneletPose();
    }

    static auto makeNativeBoundingBoxRelativeLanePosition(
      const std::string & from_entity_name, const NativeLanePosition & to_lanelet_pose,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
    {
      if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
        if (const auto from_lanelet_poses = from_entity->getCanonicalizedLaneletPoses();
            !from_lanelet_poses.empty()) {
          return makeNativeBoundingBoxRelativeLanePosition(
            // WIP just use first lanelet pose, should be changed in the future
            from_lanelet_poses.front(), from_entity->getBoundingBox(), to_lanelet_pose,
            traffic_simulator_msgs::msg::BoundingBox(), routing_algorithm);
        }
      }
      return traffic_simulator::pose::quietNaNLaneletPose();
    }

    static auto makeNativeBoundingBoxRelativeLanePosition(
      const NativeLanePosition & from_lanelet_pose,
      const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
      const NativeLanePosition & to_lanelet_pose,
      const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose
    {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      return traffic_simulator::pose::boundingBoxRelativeLaneletPose(
        from_lanelet_pose, from_bounding_box, to_lanelet_pose, to_bounding_box,
        routing_configuration);
    }

    static auto makeNativeBoundingBoxRelativeWorldPosition(
      const std::string & from_entity_name, const std::string & to_entity_name)
    {
      if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
        if (const auto to_entity = core->getEntityPointer(to_entity_name)) {
          if (
            const auto relative_pose = traffic_simulator::pose::boundingBoxRelativePose(
              from_entity->getMapPose(), from_entity->getBoundingBox(), to_entity->getMapPose(),
              to_entity->getBoundingBox())) {
            return relative_pose.value();
          }
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }

    static auto makeNativeBoundingBoxRelativeWorldPosition(
      const std::string & from_entity_name, const NativeWorldPosition & to_map_pose)
    {
      if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
        if (
          const auto relative_pose = traffic_simulator::pose::boundingBoxRelativePose(
            from_entity->getMapPose(), from_entity->getBoundingBox(), to_map_pose,
            traffic_simulator_msgs::msg::BoundingBox())) {
          return relative_pose.value();
        }
      }
      return traffic_simulator::pose::quietNaNPose();
    }

    static auto evaluateLateralRelativeLanes(
      const std::string & from_entity_name, const std::string & to_entity_name,
      const RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> int
    {
      if (const auto from_lanelet_poses =
            core->getEntity(from_entity_name).getCanonicalizedLaneletPoses();
          !from_lanelet_poses.empty()) {
        if (const auto to_lanelet_poses =
              core->getEntity(to_entity_name).getCanonicalizedLaneletPoses();
            !to_lanelet_poses.empty()) {
          traffic_simulator::RoutingConfiguration routing_configuration;
          routing_configuration.allow_lane_change =
            (routing_algorithm == RoutingAlgorithm::value_type::shortest);
          if (
            const auto lane_changes = traffic_simulator::distance::countLaneChanges(
              // WIP just use first lanelet pose, should be changed in the future
              from_lanelet_poses.front(), to_lanelet_poses.front(), routing_configuration,
              core->getHdmapUtils())) {
            return lane_changes.value().first - lane_changes.value().second;
          }
        }
      }
      throw common::Error(
        "Failed to evaluate lateral relative lanes between ", from_entity_name, " and ",
        to_entity_name);
    }
  };

  class ActionApplication  // OpenSCENARIO 1.1.1 Section 3.1.5
  {
  protected:
    template <typename... Ts>
    static auto applyAcquirePositionAction(const std::string & entity_ref, Ts &&... xs)
    {
      return core->getEntity(entity_ref).requestAcquirePosition(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyAddEntityAction(Ts &&... xs) -> decltype(auto)
    {
      return core->spawn(std::forward<decltype(xs)>(xs)...);
    }

    template <typename EntityRef, typename DynamicConstraints>
    static auto applyProfileAction(
      const EntityRef & entity_ref, const DynamicConstraints & dynamic_constraints) -> void
    {
      auto & entity = core->getEntity(entity_ref);
      entity.setBehaviorParameter([&]() {
        auto behavior_parameter = entity.getBehaviorParameter();

        // clang-format off
        const auto setIfNotInf = [](auto & target, const auto & value) {
          if (not std::isinf(value)) {
            target = value;
          }
        };
        setIfNotInf(behavior_parameter.dynamic_constraints.max_speed, dynamic_constraints.max_speed);
        setIfNotInf(behavior_parameter.dynamic_constraints.max_acceleration, dynamic_constraints.max_acceleration);
        setIfNotInf(behavior_parameter.dynamic_constraints.max_acceleration_rate, dynamic_constraints.max_acceleration_rate);
        setIfNotInf(behavior_parameter.dynamic_constraints.max_deceleration, dynamic_constraints.max_deceleration);
        setIfNotInf(behavior_parameter.dynamic_constraints.max_deceleration_rate, dynamic_constraints.max_deceleration_rate);
        // clang-format on

        return behavior_parameter;
      }());
    }

    template <typename Controller>
    static auto applyAssignControllerAction(
      const std::string & entity_ref, Controller && controller) -> void
    {
      auto & entity = core->getEntity(entity_ref);
      entity.setVelocityLimit(controller.properties.template get<Double>(
        "maxSpeed", std::numeric_limits<Double::value_type>::max()));

      entity.setBehaviorParameter([&]() {
        auto message = entity.getBehaviorParameter();
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
        core->attachImuSensor(entity_ref, [&]() {
          simulation_api_schema::ImuSensorConfiguration configuration;
          configuration.set_entity(entity_ref);
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
          configuration.set_entity(entity_ref);
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
          configuration.set_entity(entity_ref);
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
          configuration.set_entity(entity_ref);
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

        auto & ego_entity = core->getEgoEntity(entity_ref);

        if (controller.properties.contains("allowGoalModification")) {
          ego_entity.setParameter<bool>(
            "allow_goal_modification",
            controller.properties.template get<Boolean>("allowGoalModification"));
        }

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
            ego_entity.requestAutoModeForCooperation(module, false);
          } catch (const Error & error) {
            throw Error(
              "featureIdentifiersRequiringExternalPermissionForAutonomousDecisions is not "
              "supported in this environment: ",
              error.what());
          }
        }
      }
    }

    template <typename... Ts>
    static auto applyAssignRouteAction(const std::string & entity_ref, Ts &&... xs)
    {
      return core->getEntity(entity_ref).requestAssignRoute(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyDeleteEntityAction(Ts &&... xs)
    {
      return core->despawn(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyFollowTrajectoryAction(const std::string & entity_ref, Ts &&... xs)
    {
      return core->getEntity(entity_ref).requestFollowTrajectory(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyLaneChangeAction(const std::string & entity_ref, Ts &&... xs)
    {
      return core->getEntity(entity_ref).requestLaneChange(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applySpeedAction(const std::string & entity_ref, Ts &&... xs)
    {
      return core->getEntity(entity_ref).requestSpeedChange(std::forward<decltype(xs)>(xs)...);
    }

    template <
      typename PoseType, typename... Ts,
      typename = std::enable_if_t<
        std::is_same_v<std::decay_t<PoseType>, geometry_msgs::msg::Pose> ||
        std::is_same_v<std::decay_t<PoseType>, traffic_simulator::LaneletPose> ||
        std::is_same_v<std::decay_t<PoseType>, traffic_simulator::CanonicalizedLaneletPose>>>
    static auto applyTeleportAction(const std::string & name, const PoseType & pose, Ts &&... xs)
    {
      return core->getEntity(name).setStatus(pose, std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyTeleportAction(
      const std::string & name, const std::string & reference_entity_name, Ts &&... xs)
    {
      return core->getEntity(name).setStatus(
        core->getEntity(reference_entity_name).getMapPose(), std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyWalkStraightAction(const std::string & entity_ref, Ts &&... xs)
    {
      return core->getEntity(entity_ref).requestWalkStraight(std::forward<decltype(xs)>(xs)...);
    }
  };

  // OpenSCENARIO 1.1.1 Section 3.1.5
  class ConditionEvaluation : protected CoordinateSystemConversion
  {
  protected:
    static auto evaluateAcceleration(const std::string & name)
    {
      return core->getEntity(name).getCurrentAccel().linear.x;
    }

    template <typename... Ts>
    static auto evaluateCollisionCondition(Ts &&... xs) -> bool
    {
      return core->checkCollision(std::forward<decltype(xs)>(xs)...);
    }

    static auto evaluateBoundingBoxEuclideanDistance(
      const std::string & from_entity_name,
      const std::string & to_entity_name)  // for RelativeDistanceCondition
    {
      if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
        if (const auto to_entity = core->getEntityPointer(to_entity_name)) {
          if (
            const auto distance = traffic_simulator::distance::boundingBoxDistance(
              from_entity->getMapPose(), from_entity->getBoundingBox(), to_entity->getMapPose(),
              to_entity->getBoundingBox())) {
            return distance.value();
          }
        }
      }
      return std::numeric_limits<double>::quiet_NaN();
    }

    static auto evaluateRelativeSpeed(const Entity & from, const Entity & to) -> Eigen::Vector3d
    {
      if (const auto observer = core->getEntityPointer(from.name())) {
        if (const auto observed = core->getEntityPointer(to.name())) {
          auto velocity = [](const auto & entity) -> Eigen::Vector3d {
            auto direction = [](const auto & q) -> Eigen::Vector3d {
              return Eigen::Quaternion(q.w, q.x, q.y, q.z) * Eigen::Vector3d::UnitX();
            };
            return direction(entity->getMapPose().orientation) * entity->getCurrentTwist().linear.x;
          };

          const Eigen::Matrix3d rotation =
            math::geometry::getRotationMatrix(observer->getMapPose().orientation);

          return rotation.transpose() * velocity(observed) -
                 rotation.transpose() * velocity(observer);
        }
      }

      return Eigen::Vector3d(Double::nan(), Double::nan(), Double::nan());
    }

    template <typename... Ts>
    static auto evaluateSimulationTime(Ts &&... xs) -> double
    {
      if (SimulatorCore::active()) {
        return core->getCurrentTime(std::forward<decltype(xs)>(xs)...);
      } else {
        return std::numeric_limits<double>::quiet_NaN();
      }
    }

    static auto evaluateSpeed(const std::string & name)
    {
      /*
         The function name "evaluateSpeed" stands for "evaluate SpeedCondition"
         and is a part used to implement `SpeedCondition::evaluate`.
         SpeedCondition can be evaluated in three directions: longitudinal,
         lateral, and vertical, based on the attribute direction. Therefore,
         please note that this function returns velocity, that is, a vector,
         rather than speed, contrary to the name "evaluateSpeed".
      */
      const auto linear = core->getEntity(name).getCurrentTwist().linear;
      return Eigen::Vector3d(linear.x, linear.y, linear.z);
    }

    static auto evaluateStandStill(const std::string & name)
    {
      return core->getEntity(name).getStandStillDuration();
    }

    static auto evaluateTimeHeadway(
      const std::string & from_entity_name, const std::string & to_entity_name)
    {
      if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
        if (const auto to_entity = core->getEntityPointer(to_entity_name)) {
          if (const auto relative_pose = traffic_simulator::pose::relativePose(
                from_entity->getMapPose(), to_entity->getMapPose());
              relative_pose && relative_pose->position.x <= 0) {
            const double time_headway =
              (relative_pose->position.x * -1) / to_entity->getCurrentTwist().linear.x;
            return std::isnan(time_headway) ? std::numeric_limits<double>::infinity()
                                            : time_headway;
          }
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
      const std::string & entity_ref, const Performance & performance,
      const Properties & properties)
    {
      core->getEntity(entity_ref)
        .activateOutOfRangeJob(
          -performance.max_speed, +performance.max_speed, -performance.max_deceleration,
          +performance.max_acceleration,
          properties.template get<Double>("minJerk", Double::lowest()),
          properties.template get<Double>("maxJerk", Double::max()));
    }

    static auto activateNonUserDefinedControllers() -> decltype(auto)
    {
      return core->startNpcLogic();
    }

    template <typename... Ts>
    static auto evaluateCurrentState(const std::string & entity_ref, Ts &&... xs) -> decltype(auto)
    {
      return core->getEntity(entity_ref).getCurrentAction(std::forward<decltype(xs)>(xs)...);
    }

    template <typename EntityRef, typename OSCLanePosition>
    static auto evaluateRelativeHeading(
      const EntityRef & entity_ref, const OSCLanePosition & osc_lane_position)
    {
      if (const auto entity = core->getEntityPointer(entity_ref)) {
        const auto from_map_pose = entity->getMapPose();
        const auto to_map_pose = static_cast<NativeWorldPosition>(osc_lane_position);
        if (
          const auto relative_pose =
            traffic_simulator::pose::relativePose(from_map_pose, to_map_pose)) {
          return static_cast<Double>(std::abs(
            math::geometry::convertQuaternionToEulerAngle(relative_pose.value().orientation).z));
        }
      }
      return Double::nan();
    }

    template <typename EntityRef>
    static auto evaluateRelativeHeading(const EntityRef & entity_ref)
    {
      if (const auto entity = core->getEntityPointer(entity_ref)) {
        if (const auto canonicalized_lanelet_poses = entity->getCanonicalizedLaneletPoses();
            !canonicalized_lanelet_poses.empty()) {
          return static_cast<Double>(std::abs(
            // WIP just use first lanelet pose, should be changed in the future
            static_cast<traffic_simulator::LaneletPose>(canonicalized_lanelet_poses.front())
              .rpy.z));
        }
      }
      return Double::nan();
    }

    static auto engage(const std::string & ego_ref) -> decltype(auto)
    {
      return core->getEgoEntity(ego_ref).engage();
    }

    static auto isEngageable(const std::string & ego_ref) -> decltype(auto)
    {
      return core->getEgoEntity(ego_ref).isEngageable();
    }

    static auto isEngaged(const std::string & ego_ref) -> decltype(auto)
    {
      return core->getEgoEntity(ego_ref).isEngaged();
    }

    template <typename... Ts>
    static auto sendCooperateCommand(Ts &&... xs) -> decltype(auto)
    {
      /// @note here ego name is not passed from OpenScenarioInterpreter, it uses first found
      if (const auto ego_name = core->getFirstEgoName()) {
        return core->getEgoEntity(ego_name.value())
          .sendCooperateCommand(std::forward<decltype(xs)>(xs)...);
      } else {
        throw common::Error("No ego entity exists.");
      }
    }

    static auto getMinimumRiskManeuverBehaviorName(const std::string & ego_ref) -> decltype(auto)
    {
      return core->getEgoEntity(ego_ref).getMinimumRiskManeuverBehaviorName();
    }

    static auto getMinimumRiskManeuverStateName(const std::string & ego_ref) -> decltype(auto)
    {
      return core->getEgoEntity(ego_ref).getMinimumRiskManeuverStateName();
    }

    static auto getEmergencyStateName(const std::string & ego_ref) -> decltype(auto)
    {
      return core->getEgoEntity(ego_ref).getEmergencyStateName();
    }

    static auto getTurnIndicatorsCommandName(const std::string & ego_ref) -> decltype(auto)
    {
      return core->getEgoEntity(ego_ref).getTurnIndicatorsCommandName();
    }

    // TrafficLights - Conventional and V2I
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
