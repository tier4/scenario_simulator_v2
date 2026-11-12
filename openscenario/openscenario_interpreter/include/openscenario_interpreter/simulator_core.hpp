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

#include <Eigen/Core>
#include <memory>
#include <openscenario_interpreter/syntax/routing_algorithm.hpp>
#include <string>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>

namespace rclcpp_lifecycle
{
class LifecycleNode;
}  // namespace rclcpp_lifecycle

namespace traffic_simulator
{
class API;
struct Configuration;
}  // namespace traffic_simulator

namespace openscenario_interpreter
{
class EntityOperations;
class TrafficSignalOperations;

inline namespace syntax
{
struct Controller;
struct Entity;
}  // namespace syntax

using NativeWorldPosition = geometry_msgs::msg::Pose;

using NativeRelativeWorldPosition = NativeWorldPosition;

using NativeLanePosition = traffic_simulator::CanonicalizedLaneletPose;

using NativeRelativeLanePosition = traffic_simulator::LaneletPose;

class SimulatorCore
{
  friend class EntityOperations;
  friend class TrafficSignalOperations;

  static std::unique_ptr<traffic_simulator::API> core;

public:
  static auto activate(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const traffic_simulator::Configuration & configuration, double real_time_factor,
    double frame_rate) -> void;

  static auto active() -> bool;

  static auto deactivate() -> void;

  static auto update() -> void;

  class CoordinateSystemConversion
  {
  protected:
    static auto canonicalize(const traffic_simulator::LaneletPose & non_canonicalized)
      -> NativeLanePosition;

    static auto toNativeLanePosition(const NativeWorldPosition &) -> NativeLanePosition;

    static auto toNativeWorldPosition(const NativeLanePosition &) -> NativeWorldPosition;

    static auto makeNativeRelativeWorldPosition(
      const std::string & from_entity_name, const std::string & to_entity_name)
      -> NativeRelativeWorldPosition;

    static auto makeNativeRelativeWorldPosition(
      const std::string & from_entity_name, const NativeWorldPosition & to_map_pose)
      -> NativeRelativeWorldPosition;

    static auto makeNativeRelativeWorldPosition(
      const NativeWorldPosition & from_map_pose, const std::string & to_entity_name)
      -> NativeRelativeWorldPosition;

    static auto makeNativeRelativeLanePosition(
      const std::string & from_entity_name, const std::string & to_entity_name,
      RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose;

    static auto makeNativeRelativeLanePosition(
      const std::string & from_entity_name, const NativeLanePosition & to_lanelet_pose,
      RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose;

    static auto makeNativeRelativeLanePosition(
      const NativeLanePosition & from_lanelet_pose, const NativeLanePosition & to_lanelet_pose,
      RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose;

    static auto makeNativeBoundingBoxRelativeLanePosition(
      const std::string & from_entity_name, const std::string & to_entity_name,
      RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> NativeRelativeLanePosition;

    static auto makeNativeBoundingBoxRelativeLanePosition(
      const std::string & from_entity_name, const NativeLanePosition & to_lanelet_pose,
      RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> NativeRelativeLanePosition;

    static auto makeNativeBoundingBoxRelativeLanePosition(
      const NativeLanePosition & from_lanelet_pose,
      const traffic_simulator_msgs::msg::BoundingBox & from_bounding_box,
      const NativeLanePosition & to_lanelet_pose,
      const traffic_simulator_msgs::msg::BoundingBox & to_bounding_box,
      RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined)
      -> traffic_simulator::LaneletPose;

    static auto makeNativeBoundingBoxRelativeWorldPosition(
      const std::string & from_entity_name, const std::string & to_entity_name)
      -> NativeRelativeWorldPosition;

    static auto makeNativeBoundingBoxRelativeWorldPosition(
      const std::string & from_entity_name, const NativeWorldPosition & to_map_pose)
      -> NativeRelativeWorldPosition;

    static auto evaluateLateralRelativeLanes(
      const std::string & from_entity_name, const std::string & to_entity_name,
      RoutingAlgorithm::value_type routing_algorithm = RoutingAlgorithm::undefined) -> int;
  };

  class ActionApplication  // OpenSCENARIO 1.1.1 Section 3.1.5
  {
  protected:
    static auto applyAssignControllerAction(
      const std::string & entity_ref, const Controller & controller) -> void;
  };

  // OpenSCENARIO 1.1.1 Section 3.1.5
  class ConditionEvaluation : protected CoordinateSystemConversion
  {
  protected:
    static auto evaluateAcceleration(const std::string & name) -> double;

    static auto evaluateCollisionCondition(const Entity &, const Entity &) -> bool;

    static auto evaluateBoundingBoxEuclideanDistance(
      const std::string & from_entity_name, const std::string & to_entity_name) -> double;

    static auto evaluateRelativeSpeed(const Entity & from, const Entity & to) -> Eigen::Vector3d;

    static auto evaluateSimulationTime() -> double;

    static auto evaluateSpeed(const std::string & name) -> Eigen::Vector3d;

    static auto evaluateStandStill(const std::string & name) -> double;

    static auto evaluateTimeHeadway(
      const std::string & from_entity_name, const std::string & to_entity_name) -> double;
  };

  class NonStandardOperation : private CoordinateSystemConversion
  {
  protected:
    static auto activateNonUserDefinedControllers() -> void;

    static auto evaluateCurrentState(const std::string & entity_ref) -> std::string;

    static auto evaluateRelativeHeading(const Entity & entity_ref) -> double;

    static auto evaluateRelativeHeading(const std::string & entity_ref) -> double;

    static auto evaluateRelativeHeading(
      const std::string & entity_ref, const NativeWorldPosition & target) -> double;

    static auto engage(const std::string & ego_ref) -> void;

    static auto isEngageable(const std::string & ego_ref) -> bool;

    static auto isEngaged(const std::string & ego_ref) -> bool;

    static auto sendCooperateCommand(const std::string & module_name, const std::string & command)
      -> void;

    static auto getMinimumRiskManeuverBehaviorName(const std::string & ego_ref) -> std::string;

    static auto getMinimumRiskManeuverStateName(const std::string & ego_ref) -> std::string;

    static auto getEmergencyStateName(const std::string & ego_ref) -> std::string;

    static auto getTurnIndicatorsCommandName(const std::string & ego_ref) -> std::string;
  };
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_
