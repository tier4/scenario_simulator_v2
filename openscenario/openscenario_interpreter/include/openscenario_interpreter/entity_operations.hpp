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

#ifndef OPENSCENARIO_INTERPRETER__ENTITY_OPERATIONS_HPP_
#define OPENSCENARIO_INTERPRETER__ENTITY_OPERATIONS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <openscenario_interpreter/syntax/dynamic_constraints.hpp>
#include <openscenario_interpreter/syntax/performance.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <string>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/data_type/route_option.hpp>
#include <traffic_simulator/data_type/speed_change.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/misc_object_parameters.hpp>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <vector>

namespace traffic_simulator
{
class API;
}  // namespace traffic_simulator

namespace openscenario_interpreter
{
class EntityOperations
{
  static auto api() -> traffic_simulator::API &;

public:
  auto applyAcquirePositionAction(
    const std::string &, const geometry_msgs::msg::Pose &, const traffic_simulator::RouteOption &)
    -> void;

  auto applyAcquirePositionAction(
    const std::string &, const traffic_simulator::CanonicalizedLaneletPose &,
    const traffic_simulator::RouteOption &) -> void;

  auto applyAssignRouteAction(
    const std::string &, const std::vector<traffic_simulator::CanonicalizedLaneletPose> &,
    const traffic_simulator::RouteOption &) -> void;

  auto applyFollowTrajectoryAction(
    const std::string &, const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> &)
    -> void;

  auto applyLaneChangeAction(
    const std::string &, const traffic_simulator::lane_change::AbsoluteTarget &,
    traffic_simulator::lane_change::TrajectoryShape,
    const traffic_simulator::lane_change::Constraint &) -> void;

  auto applyLaneChangeAction(
    const std::string &, const traffic_simulator::lane_change::RelativeTarget &,
    traffic_simulator::lane_change::TrajectoryShape,
    const traffic_simulator::lane_change::Constraint &) -> void;

  auto applyDeleteEntityAction(const std::string &) -> void;

  auto applyAddEntityAction(
    const std::string &, const geometry_msgs::msg::Pose &,
    const traffic_simulator_msgs::msg::VehicleParameters &, const std::string &,
    const std::string &) -> void;

  auto applyAddEntityAction(
    const std::string &, const traffic_simulator::CanonicalizedLaneletPose &,
    const traffic_simulator_msgs::msg::VehicleParameters &, const std::string &,
    const std::string &) -> void;

  auto applyAddEntityAction(
    const std::string &, const geometry_msgs::msg::Pose &,
    const traffic_simulator_msgs::msg::PedestrianParameters &, const std::string &,
    const std::string &) -> void;

  auto applyAddEntityAction(
    const std::string &, const traffic_simulator::CanonicalizedLaneletPose &,
    const traffic_simulator_msgs::msg::PedestrianParameters &, const std::string &,
    const std::string &) -> void;

  auto applyAddEntityAction(
    const std::string &, const geometry_msgs::msg::Pose &,
    const traffic_simulator_msgs::msg::MiscObjectParameters &, const std::string &,
    const std::string &) -> void;

  auto applyAddEntityAction(
    const std::string &, const traffic_simulator::CanonicalizedLaneletPose &,
    const traffic_simulator_msgs::msg::MiscObjectParameters &, const std::string &,
    const std::string &) -> void;

  auto activatePerformanceAssertion(
    const std::string &, const syntax::Performance &, const syntax::Properties &) -> void;

  auto applyProfileAction(const std::string &, const syntax::DynamicConstraints &) -> void;

  auto applySpeedAction(
    const std::string &, double, traffic_simulator::speed_change::Transition,
    traffic_simulator::speed_change::Constraint, bool) -> void;

  auto applySpeedAction(
    const std::string &, const traffic_simulator::speed_change::RelativeTargetSpeed &,
    traffic_simulator::speed_change::Transition, traffic_simulator::speed_change::Constraint, bool)
    -> void;

  auto applyTeleportAction(const std::string &, const geometry_msgs::msg::Pose &) -> void;

  auto applyTeleportAction(const std::string &, const traffic_simulator::CanonicalizedLaneletPose &)
    -> void;

  auto applyTeleportAction(
    const std::string &, const std::string &, const geometry_msgs::msg::Point &,
    const geometry_msgs::msg::Vector3 &) -> void;

  auto applyWalkStraightAction(const std::string &) -> void;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__ENTITY_OPERATIONS_HPP_
