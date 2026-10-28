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
#include <limits>
#include <openscenario_interpreter/entity_operations.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <traffic_simulator/api/api.hpp>

namespace openscenario_interpreter
{
auto EntityOperations::api() -> traffic_simulator::API & { return *SimulatorCore::core; }

auto EntityOperations::applyAcquirePositionAction(
  const std::string & name, const geometry_msgs::msg::Pose & position,
  const traffic_simulator::RouteOption & options) -> void
{
  api().getEntity(name).requestAcquirePosition(position, options);
}

auto EntityOperations::applyAcquirePositionAction(
  const std::string & name, const traffic_simulator::CanonicalizedLaneletPose & position,
  const traffic_simulator::RouteOption & options) -> void
{
  api().getEntity(name).requestAcquirePosition(position, options);
}

auto EntityOperations::applyAssignRouteAction(
  const std::string & name, const std::vector<traffic_simulator::CanonicalizedLaneletPose> & route,
  const traffic_simulator::RouteOption & options) -> void
{
  api().getEntity(name).requestAssignRoute(route, options);
}

auto EntityOperations::applyFollowTrajectoryAction(
  const std::string & name,
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & trajectory) -> void
{
  /// @note add current entity pose as the first waypoint in the trajectory
  traffic_simulator_msgs::msg::Vertex initial_vertex;
  initial_vertex.time = std::numeric_limits<double>::quiet_NaN();
  initial_vertex.position = api().getEntity(name).getMapPose();
  trajectory->shape.vertices.insert(trajectory->shape.vertices.begin(), initial_vertex);
  api().getEntity(name).requestFollowTrajectory(trajectory);
}

auto EntityOperations::applyLaneChangeAction(
  const std::string & name, const traffic_simulator::lane_change::AbsoluteTarget & target,
  const traffic_simulator::lane_change::TrajectoryShape shape,
  const traffic_simulator::lane_change::Constraint & constraint) -> void
{
  api().getEntity(name).requestLaneChange(target, shape, constraint);
}

auto EntityOperations::applyLaneChangeAction(
  const std::string & name, const traffic_simulator::lane_change::RelativeTarget & target,
  const traffic_simulator::lane_change::TrajectoryShape shape,
  const traffic_simulator::lane_change::Constraint & constraint) -> void
{
  api().getEntity(name).requestLaneChange(target, shape, constraint);
}

auto EntityOperations::applyDeleteEntityAction(const std::string & name) -> void
{
  api().despawn(name);
}

auto EntityOperations::applyAddEntityAction(
  const std::string & name, const geometry_msgs::msg::Pose & position,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters, const std::string & behavior,
  const std::string & model3d) -> void
{
  api().spawn(name, position, parameters, behavior, model3d);
}

auto EntityOperations::applyAddEntityAction(
  const std::string & name, const traffic_simulator::CanonicalizedLaneletPose & position,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters, const std::string & behavior,
  const std::string & model3d) -> void
{
  api().spawn(name, position, parameters, behavior, model3d);
}

auto EntityOperations::applyAddEntityAction(
  const std::string & name, const geometry_msgs::msg::Pose & position,
  const traffic_simulator_msgs::msg::PedestrianParameters & parameters,
  const std::string & behavior, const std::string & model3d) -> void
{
  api().spawn(name, position, parameters, behavior, model3d);
}

auto EntityOperations::applyAddEntityAction(
  const std::string & name, const traffic_simulator::CanonicalizedLaneletPose & position,
  const traffic_simulator_msgs::msg::PedestrianParameters & parameters,
  const std::string & behavior, const std::string & model3d) -> void
{
  api().spawn(name, position, parameters, behavior, model3d);
}

auto EntityOperations::applyAddEntityAction(
  const std::string & name, const geometry_msgs::msg::Pose & position,
  const traffic_simulator_msgs::msg::MiscObjectParameters & parameters,
  const std::string & behavior, const std::string & model3d) -> void
{
  api().spawn(name, position, parameters, behavior, model3d);
}

auto EntityOperations::applyAddEntityAction(
  const std::string & name, const traffic_simulator::CanonicalizedLaneletPose & position,
  const traffic_simulator_msgs::msg::MiscObjectParameters & parameters,
  const std::string & behavior, const std::string & model3d) -> void
{
  api().spawn(name, position, parameters, behavior, model3d);
}

auto EntityOperations::activatePerformanceAssertion(
  const std::string & name, const syntax::Performance & performance,
  const syntax::Properties & properties) -> void
{
  api().getEntity(name).activateOutOfRangeJob(
    -performance.max_speed, performance.max_speed, -performance.max_deceleration,
    performance.max_acceleration, properties.get<Double>("minJerk", Double::lowest()),
    properties.get<Double>("maxJerk", Double::max()));
}

auto EntityOperations::applyProfileAction(
  const std::string & name, const syntax::DynamicConstraints & constraints) -> void
{
  auto & entity = api().getEntity(name);
  auto parameter = entity.getBehaviorParameter();
  const auto set_if_finite = [](auto & target, const auto & value) {
    if (not std::isinf(value)) {
      target = value;
    }
  };
  set_if_finite(parameter.dynamic_constraints.max_speed, constraints.max_speed);
  set_if_finite(parameter.dynamic_constraints.max_acceleration, constraints.max_acceleration);
  set_if_finite(
    parameter.dynamic_constraints.max_acceleration_rate, constraints.max_acceleration_rate);
  set_if_finite(parameter.dynamic_constraints.max_deceleration, constraints.max_deceleration);
  set_if_finite(
    parameter.dynamic_constraints.max_deceleration_rate, constraints.max_deceleration_rate);
  entity.setBehaviorParameter(parameter);
}

auto EntityOperations::applySpeedAction(
  const std::string & name, const double target,
  const traffic_simulator::speed_change::Transition transition,
  const traffic_simulator::speed_change::Constraint constraint, const bool continuous) -> void
{
  api().getEntity(name).requestSpeedChange(target, transition, constraint, continuous);
}

auto EntityOperations::applySpeedAction(
  const std::string & name, const traffic_simulator::speed_change::RelativeTargetSpeed & target,
  const traffic_simulator::speed_change::Transition transition,
  const traffic_simulator::speed_change::Constraint constraint, const bool continuous) -> void
{
  api().getEntity(name).requestSpeedChange(target, transition, constraint, continuous);
}

auto EntityOperations::applyTeleportAction(
  const std::string & name, const geometry_msgs::msg::Pose & position) -> void
{
  api().getEntity(name).setStatus(position);
}

auto EntityOperations::applyTeleportAction(
  const std::string & name, const traffic_simulator::CanonicalizedLaneletPose & position) -> void
{
  api().getEntity(name).setStatus(position);
}

auto EntityOperations::applyTeleportAction(
  const std::string & name, const std::string & reference_name,
  const geometry_msgs::msg::Point & position, const geometry_msgs::msg::Vector3 & orientation)
  -> void
{
  api().getEntity(name).setStatus(
    api().getEntity(reference_name).getMapPose(), position, orientation);
}

auto EntityOperations::applyWalkStraightAction(const std::string & name) -> void
{
  api().getEntity(name).requestWalkStraight();
}
}  // namespace openscenario_interpreter
