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

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <limits>
#include <memory>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/unsigned_integer.hpp>
#include <openscenario_interpreter/type_traits/requires.hpp>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>
#include <utility>

namespace openscenario_interpreter
{
using NativeWorldPosition = geometry_msgs::msg::Pose;

using NativeRelativeWorldPosition = NativeWorldPosition;

using NativeLanePosition = traffic_simulator_msgs::msg::LaneletPose;

using NativeRelativeLanePosition = NativeLanePosition;

class SimulatorCore
{
  static inline std::unique_ptr<traffic_simulator::API> core = nullptr;

public:
  template <typename Node, typename... Ts>
  static auto activate(
    const Node & node, const traffic_simulator::Configuration & configuration, Ts &&... xs) -> void
  {
    if (not active()) {
      core = std::make_unique<traffic_simulator::API>(node, configuration);
      core->initialize(std::forward<decltype(xs)>(xs)...);
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
    template <typename T, typename std::enable_if_t<std::is_same_v<T, NativeLanePosition>, int> = 0>
    static auto convert(const geometry_msgs::msg::Pose & pose)
    {
      if (const auto result = core->toLaneletPose(pose, false); result) {
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
    static auto convert(const NativeLanePosition & native_lane_position)
    {
      return core->toMapPose(native_lane_position);
    }

    template <typename OSCLanePosition>
    static auto makeNativeLanePosition(const OSCLanePosition & osc_lane_position)
    {
      NativeLanePosition native_lane_position;
      native_lane_position.lanelet_id =
        boost::lexical_cast<std::int64_t>(osc_lane_position.lane_id);
      native_lane_position.s = osc_lane_position.s;
      native_lane_position.offset = osc_lane_position.offset;
      native_lane_position.rpy.x = osc_lane_position.orientation.r;
      native_lane_position.rpy.y = osc_lane_position.orientation.p;
      native_lane_position.rpy.z = osc_lane_position.orientation.h;
      return native_lane_position;
    }

    template <typename OSCWorldPosition>
    static auto makeNativeWorldPosition(const OSCWorldPosition & osc_world_position)
    {
      NativeWorldPosition native_world_position;
      native_world_position.position.x = osc_world_position.x;
      native_world_position.position.y = osc_world_position.y;
      native_world_position.position.z = osc_world_position.z;
      native_world_position.orientation =
        quaternion_operation::convertEulerAngleToQuaternion([&]() {
          geometry_msgs::msg::Vector3 vector;
          vector.x = osc_world_position.r;
          vector.y = osc_world_position.p;
          vector.z = osc_world_position.h;
          return vector;
        }());
      return native_world_position;
    }

    template <typename... Ts>
    static auto makeNativeRelativeWorldPosition(Ts &&... xs)
    {
      try {
        return SimulatorCore::core->getRelativePose(std::forward<decltype(xs)>(xs)...);
      } catch (...) {
        geometry_msgs::msg::Pose result{};
        result.position.x = std::numeric_limits<double>::quiet_NaN();
        result.position.y = std::numeric_limits<double>::quiet_NaN();
        result.position.z = std::numeric_limits<double>::quiet_NaN();
        result.orientation.x = 0;
        result.orientation.y = 0;
        result.orientation.z = 0;
        result.orientation.w = 1;
        return result;
      }
    }

    template <typename From, typename To>
    static auto makeNativeRelativeLanePosition(const From & from, const To & to)
    {
      auto s = [](auto &&... xs) {
        if (const auto result = core->getLongitudinalDistance(std::forward<decltype(xs)>(xs)...);
            result) {
          return result.value();
        } else {
          return std::numeric_limits<
            typename std::decay_t<decltype(result)>::value_type>::quiet_NaN();
        }
      };

      auto t = [](auto &&... xs) {
        if (const auto result = core->getLateralDistance(std::forward<decltype(xs)>(xs)...);
            result) {
          return *result;
        } else {
          return std::numeric_limits<
            typename std::decay_t<decltype(result)>::value_type>::quiet_NaN();
        }
      };

      NativeRelativeLanePosition position;
      position.lanelet_id = std::numeric_limits<std::int64_t>::max();
      position.s = s(from, to);
      position.offset = t(from, to);
      position.rpy.x = std::numeric_limits<double>::quiet_NaN();
      position.rpy.y = std::numeric_limits<double>::quiet_NaN();
      position.rpy.z = std::numeric_limits<double>::quiet_NaN();
      return position;
    }
  };

  class ActionApplication  // OpenSCENARIO 1.1.1 Section 3.1.5
  {
  protected:
    template <typename... Ts>
    static auto applyAcquirePositionAction(Ts &&... xs)
    {
      return core->requestAcquirePosition(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyAddEntityAction(Ts &&... xs)
    {
      return core->spawn(std::forward<decltype(xs)>(xs)...);
    }

    template <typename EntityRef, typename DynamicConstraints>
    static auto applyProfileAction(
      const EntityRef & entity_ref, const DynamicConstraints & dynamic_constraints) -> void
    {
      return core->setBehaviorParameter(entity_ref, [&]() {
        auto behavior_parameter = core->getBehaviorParameter(entity_ref);

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
      const std::string & entity_ref, Controller && controller) -> void
    {
      core->setVelocityLimit(
        entity_ref, controller.properties.template get<Double>(
                      "maxSpeed", std::numeric_limits<Double::value_type>::max()));

      core->setBehaviorParameter(entity_ref, [&]() {
        auto message = core->getBehaviorParameter(entity_ref);
        message.see_around = not controller.properties.template get<Boolean>("isBlind");
        return message;
      }());

      if (controller.isUserDefinedController()) {
        core->attachLidarSensor(
          entity_ref, controller.properties.template get<Double>("pointcloudPublishingDelay"));

        core->attachDetectionSensor([&]() {
          simulation_api_schema::DetectionSensorConfiguration configuration;
          // clang-format off
          configuration.set_architecture_type(getParameter<std::string>("architecture_type", "awf/universe"));
          configuration.set_entity(entity_ref);
          configuration.set_filter_by_range(controller.properties.template get<Boolean>("isClairvoyant"));
          configuration.set_object_recognition_delay(controller.properties.template get<Double>("detectedObjectPublishingDelay"));
          configuration.set_pos_noise_stddev(controller.properties.template get<Double>("detectedObjectPositionStandardDeviation"));
          configuration.set_probability_of_lost(controller.properties.template get<Double>("detectedObjectMissingProbability"));
          configuration.set_random_seed(controller.properties.template get<UnsignedInteger>("randomSeed"));
          configuration.set_range(300);
          configuration.set_update_duration(0.1);
          // clang-format on
          return configuration;
        }());

        core->attachOccupancyGridSensor([&]() {
          simulation_api_schema::OccupancyGridSensorConfiguration configuration;
          // clang-format off
          configuration.set_architecture_type(getParameter<std::string>("architecture_type", "awf/universe"));
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

        core->asFieldOperatorApplication(entity_ref)
          .setCooperator(controller.properties.template get<String>("cooperator", "simulator"));
      }
    }

    template <typename... Ts>
    static auto applyAssignRouteAction(Ts &&... xs)
    {
      return core->requestAssignRoute(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyDeleteEntityAction(Ts &&... xs)
    {
      return core->despawn(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyFollowTrajectoryAction(Ts &&... xs)
    {
      return core->requestFollowTrajectory(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyLaneChangeAction(Ts &&... xs)
    {
      return core->requestLaneChange(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applySpeedAction(Ts &&... xs)
    {
      return core->requestSpeedChange(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyTeleportAction(Ts &&... xs)
    {
      return core->setEntityStatus(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyWalkStraightAction(Ts &&... xs)
    {
      return core->requestWalkStraight(std::forward<decltype(xs)>(xs)...);
    }
  };

  // OpenSCENARIO 1.1.1 Section 3.1.5
  class ConditionEvaluation : protected CoordinateSystemConversion
  {
  protected:
    template <typename... Ts>
    static auto evaluateAcceleration(Ts &&... xs)
    {
      return core->getEntityStatus(std::forward<decltype(xs)>(xs)...).action_status.accel.linear.x;
    }

    template <typename... Ts>
    static auto evaluateCollisionCondition(Ts &&... xs) -> bool
    {
      return core->checkCollision(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto evaluateFreespaceEuclideanDistance(Ts &&... xs)  // for RelativeDistanceCondition
    {
      if (const auto result = core->getBoundingBoxDistance(std::forward<decltype(xs)>(xs)...);
          result) {
        return result.value();
      } else {
        using value_type = typename std::decay<decltype(result)>::type::value_type;
        return std::numeric_limits<value_type>::quiet_NaN();
      }
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

    template <typename... Ts>
    static auto evaluateSpeed(Ts &&... xs)
    {
      return core->getEntityStatus(std::forward<decltype(xs)>(xs)...).action_status.twist.linear.x;
    }

    template <typename... Ts>
    static auto evaluateStandStill(Ts &&... xs)
    {
      return core->getStandStillDuration(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto evaluateTimeHeadway(Ts &&... xs)
    {
      if (const auto result = core->getTimeHeadway(std::forward<decltype(xs)>(xs)...); result) {
        return result.value();
      } else {
        using value_type = typename std::decay<decltype(result)>::type::value_type;
        return std::numeric_limits<value_type>::quiet_NaN();
      }
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
      core->activateOutOfRangeJob(
        entity_ref, -performance.max_speed, +performance.max_speed, -performance.max_deceleration,
        +performance.max_acceleration, properties.template get<Double>("minJerk", Double::lowest()),
        properties.template get<Double>("maxJerk", Double::max()));
    }

    template <typename... Ts>
    static auto asFieldOperatorApplication(Ts &&... xs) -> decltype(auto)
    {
      return core->asFieldOperatorApplication(std::forward<decltype(xs)>(xs)...);
    }

    static auto activateNonUserDefinedControllers() -> decltype(auto)
    {
      return core->startNpcLogic();
    }

    template <typename... Ts>
    static auto evaluateCurrentState(Ts &&... xs) -> decltype(auto)
    {
      return core->getCurrentAction(std::forward<decltype(xs)>(xs)...);
    }

    template <typename EntityRef, typename OSCLanePosition>
    static auto evaluateRelativeHeading(
      const EntityRef & entity_ref, const OSCLanePosition & osc_lane_position)
    {
      return std::abs(
        quaternion_operation::convertQuaternionToEulerAngle(
          core->getRelativePose(entity_ref, makeNativeLanePosition(osc_lane_position)).orientation)
          .z);
    }

    template <typename EntityRef>
    static auto evaluateRelativeHeading(const EntityRef & entity_ref)
    {
      if (auto entity_status = core->getEntityStatus(entity_ref);
          entity_status.lanelet_pose_valid) {
        return static_cast<Double>(std::abs(entity_status.lanelet_pose.rpy.z));
      } else {
        return Double::nan();
      }
    }

    template <typename... Ts>
    static auto getConventionalTrafficLights(Ts &&... xs) -> decltype(auto)
    {
      return core->getConventionalTrafficLights(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto getV2ITrafficLights(Ts &&... xs) -> decltype(auto)
    {
      return core->getV2ITrafficLights(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto resetConventionalTrafficLightPublishRate(Ts &&... xs) -> decltype(auto)
    {
      return core->resetConventionalTrafficLightPublishRate(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto resetV2ITrafficLightPublishRate(Ts &&... xs) -> decltype(auto)
    {
      return core->resetV2ITrafficLightPublishRate(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto sendCooperateCommand(Ts &&... xs) -> decltype(auto)
    {
      return asFieldOperatorApplication(core->getEgoName())
        .sendCooperateCommand(std::forward<decltype(xs)>(xs)...);
    }
  };
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_
