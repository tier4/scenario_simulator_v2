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

#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cctype>
#include <cmath>
#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <get_parameter/get_parameter.hpp>
#include <limits>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/unsigned_integer.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sstream>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator/utils/route.hpp>
#include <vector>

namespace openscenario_interpreter
{
std::unique_ptr<traffic_simulator::API> SimulatorCore::core = nullptr;

auto SimulatorCore::activate(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
  const traffic_simulator::Configuration & configuration, const double real_time_factor,
  const double frame_rate) -> void
{
  if (not active()) {
    core =
      std::make_unique<traffic_simulator::API>(node, configuration, real_time_factor, frame_rate);
  } else {
    throw Error("The simulator core has already been instantiated.");
  }
}

auto SimulatorCore::active() -> bool { return static_cast<bool>(core); }

auto SimulatorCore::deactivate() -> void
{
  if (active()) {
    core->despawnEntities();
    core->closeZMQConnection();
    core.reset();
  }
}

auto SimulatorCore::update() -> void { core->updateFrame(); }

auto SimulatorCore::CoordinateSystemConversion::canonicalize(
  const traffic_simulator::LaneletPose & non_canonicalized) -> NativeLanePosition
{
  return NativeLanePosition(non_canonicalized);
}

auto SimulatorCore::CoordinateSystemConversion::toNativeLanePosition(
  const NativeWorldPosition & pose) -> NativeLanePosition
{
  constexpr bool include_crosswalk{false};
  if (
    const auto result =
      traffic_simulator::pose::toCanonicalizedLaneletPose(pose, include_crosswalk)) {
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

auto SimulatorCore::CoordinateSystemConversion::toNativeWorldPosition(
  const NativeLanePosition & native_lane_position) -> NativeWorldPosition
{
  return traffic_simulator::pose::toMapPose(native_lane_position);
}

auto SimulatorCore::CoordinateSystemConversion::makeNativeRelativeWorldPosition(
  const std::string & from_entity_name, const std::string & to_entity_name)
  -> NativeRelativeWorldPosition
{
  if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
    if (const auto to_entity = core->getEntityPointer(to_entity_name)) {
      if (
        const auto relative_pose =
          traffic_simulator::pose::relativePose(from_entity->getMapPose(), to_entity->getMapPose()))
        return relative_pose.value();
    }
  }
  return traffic_simulator::pose::quietNaNPose();
}

auto SimulatorCore::CoordinateSystemConversion::makeNativeRelativeWorldPosition(
  const std::string & from_entity_name, const NativeWorldPosition & to_map_pose)
  -> NativeRelativeWorldPosition
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

auto SimulatorCore::CoordinateSystemConversion::makeNativeRelativeWorldPosition(
  const NativeWorldPosition & from_map_pose, const std::string & to_entity_name)
  -> NativeRelativeWorldPosition
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

auto SimulatorCore::CoordinateSystemConversion::makeNativeRelativeLanePosition(
  const std::string & from_entity_name, const std::string & to_entity_name,
  const RoutingAlgorithm::value_type routing_algorithm) -> traffic_simulator::LaneletPose
{
  if (const auto to_entity = core->getEntityPointer(to_entity_name)) {
    if (const auto to_lanelet_pose = to_entity->getCanonicalizedLaneletPose()) {
      return makeNativeRelativeLanePosition(
        from_entity_name, to_lanelet_pose.value(), routing_algorithm);
    }
  }
  return traffic_simulator::pose::quietNaNLaneletPose();
}

auto SimulatorCore::CoordinateSystemConversion::makeNativeRelativeLanePosition(
  const std::string & from_entity_name, const NativeLanePosition & to_lanelet_pose,
  const RoutingAlgorithm::value_type routing_algorithm) -> traffic_simulator::LaneletPose
{
  if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
    if (const auto from_lanelet_pose = from_entity->getCanonicalizedLaneletPose()) {
      return makeNativeRelativeLanePosition(
        from_lanelet_pose.value(), to_lanelet_pose, routing_algorithm);
    }
  }
  return traffic_simulator::pose::quietNaNLaneletPose();
}

auto SimulatorCore::CoordinateSystemConversion::makeNativeRelativeLanePosition(
  const NativeLanePosition & from_lanelet_pose, const NativeLanePosition & to_lanelet_pose,
  const RoutingAlgorithm::value_type routing_algorithm) -> traffic_simulator::LaneletPose
{
  traffic_simulator::RoutingConfiguration routing_configuration;
  routing_configuration.allow_lane_change =
    (routing_algorithm == RoutingAlgorithm::value_type::shortest);
  return traffic_simulator::pose::relativeLaneletPose(
    from_lanelet_pose, to_lanelet_pose, routing_configuration);
}

auto SimulatorCore::CoordinateSystemConversion::makeNativeBoundingBoxRelativeLanePosition(
  const std::string & from_entity_name, const std::string & to_entity_name,
  const RoutingAlgorithm::value_type routing_algorithm) -> NativeRelativeLanePosition
{
  if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
    if (const auto to_entity = core->getEntityPointer(to_entity_name)) {
      if (const auto from_lanelet_pose = from_entity->getCanonicalizedLaneletPose()) {
        if (const auto to_lanelet_pose = to_entity->getCanonicalizedLaneletPose()) {
          return makeNativeBoundingBoxRelativeLanePosition(
            from_lanelet_pose.value(), from_entity->getBoundingBox(), to_lanelet_pose.value(),
            to_entity->getBoundingBox(), routing_algorithm);
        }
      }
    }
  }
  return traffic_simulator::pose::quietNaNLaneletPose();
}

auto SimulatorCore::CoordinateSystemConversion::makeNativeBoundingBoxRelativeLanePosition(
  const std::string & from_entity_name, const NativeLanePosition & to_lanelet_pose,
  const RoutingAlgorithm::value_type routing_algorithm) -> NativeRelativeLanePosition
{
  if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
    if (const auto from_lanelet_pose = from_entity->getCanonicalizedLaneletPose()) {
      return makeNativeBoundingBoxRelativeLanePosition(
        from_lanelet_pose.value(), from_entity->getBoundingBox(), to_lanelet_pose,
        traffic_simulator_msgs::msg::BoundingBox(), routing_algorithm);
    }
  }
  return traffic_simulator::pose::quietNaNLaneletPose();
}

auto SimulatorCore::CoordinateSystemConversion::makeNativeBoundingBoxRelativeLanePosition(
  const NativeLanePosition & from_lanelet_pose,
  const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
  const NativeLanePosition & to_lanelet_pose,
  const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box,
  const RoutingAlgorithm::value_type routing_algorithm) -> traffic_simulator::LaneletPose
{
  traffic_simulator::RoutingConfiguration routing_configuration;
  routing_configuration.allow_lane_change =
    (routing_algorithm == RoutingAlgorithm::value_type::shortest);
  return traffic_simulator::pose::boundingBoxRelativeLaneletPose(
    from_lanelet_pose, from_bounding_box, to_lanelet_pose, to_bounding_box, routing_configuration);
}

auto SimulatorCore::CoordinateSystemConversion::makeNativeBoundingBoxRelativeWorldPosition(
  const std::string & from_entity_name, const std::string & to_entity_name)
  -> NativeRelativeWorldPosition
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

auto SimulatorCore::CoordinateSystemConversion::makeNativeBoundingBoxRelativeWorldPosition(
  const std::string & from_entity_name, const NativeWorldPosition & to_map_pose)
  -> NativeRelativeWorldPosition
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

auto SimulatorCore::CoordinateSystemConversion::evaluateLateralRelativeLanes(
  const std::string & from_entity_name, const std::string & to_entity_name,
  const RoutingAlgorithm::value_type routing_algorithm) -> int
{
  if (
    const auto from_lanelet_pose =
      core->getEntity(from_entity_name).getCanonicalizedLaneletPose()) {
    if (
      const auto to_lanelet_pose = core->getEntity(to_entity_name).getCanonicalizedLaneletPose()) {
      traffic_simulator::RoutingConfiguration routing_configuration;
      routing_configuration.allow_lane_change =
        (routing_algorithm == RoutingAlgorithm::value_type::shortest);
      if (
        const auto lane_changes = traffic_simulator::route::countLaneChanges(
          from_lanelet_pose.value(), to_lanelet_pose.value(), routing_configuration)) {
        return lane_changes.value().first - lane_changes.value().second;
      }
    }
  }
  throw common::Error(
    "Failed to evaluate lateral relative lanes between ", from_entity_name, " and ",
    to_entity_name);
}

auto SimulatorCore::ActionApplication::applyAssignControllerAction(
  const std::string & entity_ref, const Controller & controller) -> void
{
  auto & entity = core->getEntity(entity_ref);
  entity.setVelocityLimit(
    controller.properties.get<Double>("maxSpeed", std::numeric_limits<Double::value_type>::max()));

  if (controller.properties.contains("lateralCollisionThreshold")) {
    entity.setLateralCollisionThreshold(
      controller.properties.get<Double>("lateralCollisionThreshold"));
  } else {
    entity.setLateralCollisionThreshold(std::nullopt);
  }
  entity.setBehaviorParameter([&]() {
    auto message = entity.getBehaviorParameter();
    message.see_around = not controller.properties.get<Boolean>("isBlind");
    /// The default values written in
    /// https://github.com/tier4/scenario_simulator_v2/blob/master/simulation/traffic_simulator_msgs/msg/DynamicConstraints.msg
    message.dynamic_constraints.max_acceleration =
      controller.properties.get<Double>("maxAcceleration", 10.0);
    message.dynamic_constraints.max_acceleration_rate =
      controller.properties.get<Double>("maxAccelerationRate", 3.0);
    message.dynamic_constraints.max_deceleration =
      controller.properties.get<Double>("maxDeceleration", 10.0);
    message.dynamic_constraints.max_deceleration_rate =
      controller.properties.get<Double>("maxDecelerationRate", 3.0);
    message.dynamic_constraints.max_speed = controller.properties.get<Double>("maxSpeed", 50.0);
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
      configuration.set_horizontal_resolution(degree_to_radian(controller.properties.get<Double>("pointcloudHorizontalResolution", 1.0)));
      configuration.set_lidar_sensor_delay(controller.properties.get<Double>("pointcloudPublishingDelay"));
      configuration.set_scan_duration(0.1);
      // clang-format on

      const auto vertical_field_of_view =
        degree_to_radian(controller.properties.get<Double>("pointcloudVerticalFieldOfView", 30.0));

      const auto channels = controller.properties.get<UnsignedInteger>("pointcloudChannels", 16);

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
      configuration.set_detect_all_objects_in_range(controller.properties.get<Boolean>("isClairvoyant"));
      configuration.set_object_recognition_delay(controller.properties.get<Double>("detectedObjectPublishingDelay"));
      configuration.set_pos_noise_stddev(controller.properties.get<Double>("detectedObjectPositionStandardDeviation"));
      configuration.set_probability_of_lost(controller.properties.get<Double>("detectedObjectMissingProbability"));
      configuration.set_random_seed(controller.properties.get<UnsignedInteger>("randomSeed"));
      configuration.set_range(controller.properties.get<Double>("detectionSensorRange",300.0));
      configuration.set_object_recognition_ground_truth_delay(controller.properties.get<Double>("detectedObjectGroundTruthPublishingDelay"));
      configuration.set_update_duration(0.1);
      // clang-format on
      return configuration;
    }());

    core->attachOccupancyGridSensor([&]() {
      simulation_api_schema::OccupancyGridSensorConfiguration configuration;
      // clang-format off
      configuration.set_architecture_type(common::getParameter<std::string>("architecture_type", "awf/universe/20240605"));
      configuration.set_entity(entity_ref);
      configuration.set_filter_by_range(controller.properties.get<Boolean>("isClairvoyant"));
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
        "allow_goal_modification", controller.properties.get<Boolean>("allowGoalModification"));
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
         }(controller.properties.get<String>(
           "featureIdentifiersRequiringExternalPermissionForAutonomousDecisio"
           "ns"))) {
      try {
        ego_entity.requestAutoModeForCooperation(module, false);
      } catch (const Error & error) {
        throw Error(
          "featureIdentifiersRequiringExternalPermissionForAutonomous"
          "Decisions is not "
          "supported in this environment: ",
          error.what());
      }
    }
  }
}

auto SimulatorCore::ConditionEvaluation::evaluateAcceleration(const std::string & name) -> double
{
  return core->getEntity(name).getCurrentAccel().linear.x;
}

auto SimulatorCore::ConditionEvaluation::evaluateCollisionCondition(
  const Entity & first, const Entity & second) -> bool
{
  return core->checkCollision(first.name(), second.name());
}

auto SimulatorCore::ConditionEvaluation::evaluateBoundingBoxEuclideanDistance(
  const std::string & from_entity_name, const std::string & to_entity_name) -> double
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

auto SimulatorCore::ConditionEvaluation::evaluateRelativeSpeed(
  const Entity & from, const Entity & to) -> Eigen::Vector3d
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

      return rotation.transpose() * velocity(observed) - rotation.transpose() * velocity(observer);
    }
  }

  return Eigen::Vector3d(Double::nan(), Double::nan(), Double::nan());
}

auto SimulatorCore::ConditionEvaluation::evaluateSimulationTime() -> double
{
  if (SimulatorCore::active()) {
    return core->getCurrentTime();
  } else {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

auto SimulatorCore::ConditionEvaluation::evaluateSpeed(const std::string & name) -> Eigen::Vector3d
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

auto SimulatorCore::ConditionEvaluation::evaluateStandStill(const std::string & name) -> double
{
  return core->getEntity(name).getStandStillDuration();
}

auto SimulatorCore::ConditionEvaluation::evaluateTimeHeadway(
  const std::string & from_entity_name, const std::string & to_entity_name) -> double
{
  if (const auto from_entity = core->getEntityPointer(from_entity_name)) {
    if (const auto to_entity = core->getEntityPointer(to_entity_name)) {
      if (const auto relative_pose = traffic_simulator::pose::relativePose(
            from_entity->getMapPose(), to_entity->getMapPose());
          relative_pose && relative_pose->position.x <= 0) {
        const double time_headway =
          (relative_pose->position.x * -1) / to_entity->getCurrentTwist().linear.x;
        return std::isnan(time_headway) ? std::numeric_limits<double>::infinity() : time_headway;
      }
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

auto SimulatorCore::NonStandardOperation::activateNonUserDefinedControllers() -> void
{
  return core->startNpcLogic();
}

auto SimulatorCore::NonStandardOperation::evaluateCurrentState(const std::string & entity_ref)
  -> std::string
{
  return core->getEntity(entity_ref).getCurrentAction();
}

auto SimulatorCore::NonStandardOperation::evaluateRelativeHeading(const Entity & entity_ref)
  -> double
{
  return evaluateRelativeHeading(entity_ref.name());
}

auto SimulatorCore::NonStandardOperation::evaluateRelativeHeading(const std::string & entity_ref)
  -> double
{
  if (const auto entity = core->getEntityPointer(entity_ref)) {
    if (const auto canonicalized_lanelet_pose = entity->getCanonicalizedLaneletPose()) {
      return std::abs(
        static_cast<traffic_simulator::LaneletPose>(canonicalized_lanelet_pose.value()).rpy.z);
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

auto SimulatorCore::NonStandardOperation::evaluateRelativeHeading(
  const std::string & entity_ref, const NativeWorldPosition & target) -> double
{
  if (const auto entity = core->getEntityPointer(entity_ref)) {
    const auto from_map_pose = entity->getMapPose();
    if (const auto relative_pose = traffic_simulator::pose::relativePose(from_map_pose, target)) {
      return std::abs(
        math::geometry::convertQuaternionToEulerAngle(relative_pose.value().orientation).z);
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

auto SimulatorCore::NonStandardOperation::engage(const std::string & ego_ref) -> void
{
  return core->getEgoEntity(ego_ref).engage();
}

auto SimulatorCore::NonStandardOperation::isEngageable(const std::string & ego_ref) -> bool
{
  return core->getEgoEntity(ego_ref).isEngageable();
}

auto SimulatorCore::NonStandardOperation::isEngaged(const std::string & ego_ref) -> bool
{
  return core->getEgoEntity(ego_ref).isEngaged();
}

auto SimulatorCore::NonStandardOperation::sendCooperateCommand(
  const std::string & module_name, const std::string & command) -> void
{
  /// @note here ego name is not passed from OpenScenarioInterpreter, it uses
  /// first found
  if (const auto ego_name = core->getFirstEgoName()) {
    return core->getEgoEntity(ego_name.value()).sendCooperateCommand(module_name, command);
  } else {
    throw common::Error("No ego entity exists.");
  }
}

auto SimulatorCore::NonStandardOperation::getMinimumRiskManeuverBehaviorName(
  const std::string & ego_ref) -> std::string
{
  return core->getEgoEntity(ego_ref).getMinimumRiskManeuverBehaviorName();
}

auto SimulatorCore::NonStandardOperation::getMinimumRiskManeuverStateName(
  const std::string & ego_ref) -> std::string
{
  return core->getEgoEntity(ego_ref).getMinimumRiskManeuverStateName();
}

auto SimulatorCore::NonStandardOperation::getEmergencyStateName(const std::string & ego_ref)
  -> std::string
{
  return core->getEgoEntity(ego_ref).getEmergencyStateName();
}

auto SimulatorCore::NonStandardOperation::getTurnIndicatorsCommandName(const std::string & ego_ref)
  -> std::string
{
  return core->getEgoEntity(ego_ref).getTurnIndicatorsCommandName();
}
}  // namespace openscenario_interpreter
