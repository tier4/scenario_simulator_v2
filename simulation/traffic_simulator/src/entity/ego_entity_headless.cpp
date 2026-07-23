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

// Headless implementation of traffic_simulator::entity::EgoEntity, selected in CMake when
// SSV2_HEADLESS_EGO is defined (the non-headless implementation lives in ego_entity.cpp).
//
// This variant removes all concealer / FieldOperatorApplication coupling: there is no Autoware
// launch, no ZeroMQ, no ROS control loop. The ego is driven in-process by a Diffusion-Planner
// trajectory injected through setDiffusionTrajectory() and integrated by a
// SimModelPerfectTrajectoryTracker (wired into onUpdate at R1c). All Autoware-facing methods are
// stubbed. The class keeps the exact type name and public signatures of the full EgoEntity so
// that every dynamic_cast<EgoEntity*> / is<EgoEntity> reference elsewhere compiles unchanged.
//
// R1b scope: make the traffic_simulator library compile under -Werror with SSV2_HEADLESS_EGO ON.
// onUpdate and routing are intentionally minimal here; functional in-process driving is R1c.

#include <algorithm>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <cctype>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/quaternion/operator.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iomanip>
#include <memory>
#include <optional>
#include <string>
#include <traffic_simulator/entity/ego_entity.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator/utils/route.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <vector>

namespace
{
// rclcpp node names must match [A-Za-z_][A-Za-z0-9_]* ; sanitize the entity name into a valid one.
auto makeNodeName(const std::string & entity_name) -> std::string
{
  std::string sanitized = "ego_entity";
  for (const char c : entity_name) {
    sanitized += (std::isalnum(static_cast<unsigned char>(c)) || c == '_') ? c : '_';
  }
  return sanitized;
}
}  // namespace

namespace traffic_simulator
{
namespace entity
{
EgoEntity::EgoEntity(
  const std::string & name, const CanonicalizedEntityStatus & entity_status,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters,
  const Configuration & configuration,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters)
: VehicleEntity(name, entity_status, parameters), rclcpp::Node(makeNodeName(name), rclcpp::NodeOptions())
{
  // Autoware launch / concealer configuration is intentionally absent in the headless build.
  static_cast<void>(configuration);
  static_cast<void>(node_parameters);
  turn_indicators_command_.command = autoware_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE;

  // Build the in-process vehicle model anchored at the spawn pose (fixed initial-frame reference).
  // status_ is already populated by the VehicleEntity/EntityBase base constructors.
  vehicle_model_ = std::make_unique<SimModelPerfectTrajectoryTracker>(0.0 /* no delay */);
  initial_pose_ = status_->getMapPose();
  initial_rotation_matrix_ = math::geometry::getRotationMatrix(initial_pose_.orientation);
  vehicle_model_->setInitialReference(initial_pose_, initial_rotation_matrix_);
  // The ego status is authored in-process from here on; bypass the post-scenario setStatus guard.
  is_controlled_by_simulator_ = true;
  initialized_ = true;
}

auto EgoEntity::engage() -> void {}

auto EgoEntity::isEngaged() const -> bool { return true; }

auto EgoEntity::isEngageable() const -> bool { return true; }

auto EgoEntity::sendCooperateCommand(const std::string &, const std::string &) -> void {}

auto EgoEntity::requestAutoModeForCooperation(const std::string &, bool) -> void {}

auto EgoEntity::getMinimumRiskManeuverBehaviorName() const -> std::string { return ""; }

auto EgoEntity::getMinimumRiskManeuverStateName() const -> std::string { return ""; }

auto EgoEntity::getEmergencyStateName() const -> std::string { return ""; }

auto EgoEntity::getTurnIndicatorsCommandName() const -> std::string
{
  switch (turn_indicators_command_.command) {
    case autoware_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE:
      return "DISABLE";
    case autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT:
      return "ENABLE_LEFT";
    case autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT:
      return "ENABLE_RIGHT";
    case autoware_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND:
      return "NO_COMMAND";
    default:
      return "";
  }
}

auto EgoEntity::getCurrentAction() const -> std::string { return ""; }

auto EgoEntity::getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter
{
  return behavior_parameter_;
}

auto EgoEntity::getEntityTypename() const -> const std::string &
{
  static const std::string result = "EgoEntity";
  return result;
}

auto EgoEntity::getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return std::nullopt;
}

auto EgoEntity::getRouteLanelets(double /*unused horizon*/) -> lanelet::Ids
{
  // Headless: no Autoware path. Route lanelets are supplied by the route sidecar at R1c.
  return lanelet::Ids{};
}

auto EgoEntity::getCurrentPose() const -> const geometry_msgs::msg::Pose &
{
  return status_->getMapPose();
}

auto EgoEntity::getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray
{
  return traffic_simulator_msgs::msg::WaypointsArray{};
}

auto EgoEntity::updateFieldOperatorApplication() -> void {}

auto EgoEntity::isTeleportRequested() const -> bool { return teleport_requested_; }

auto EgoEntity::checkAndTriggerStuckJump() -> void { teleport_requested_ = false; }

void EgoEntity::onUpdate(double current_time, double step_time)
{
  using math::geometry::convertEulerAngleToQuaternion;
  using math::geometry::operator*;

  EntityBase::onUpdate(current_time, step_time);

  // 1. Current map pose -> initial-frame relative position (z preserved through the R^T/R roundtrip;
  //    SimModelInterface is 2D so z is carried outside the model state).
  const auto & map_pose = status_->getMapPose();
  world_relative_position_ = initial_rotation_matrix_.transpose() *
                             Eigen::Vector3d(
                               map_pose.position.x - initial_pose_.position.x,
                               map_pose.position.y - initial_pose_.position.y,
                               map_pose.position.z - initial_pose_.position.z);

  // 2. Step the vehicle model with the injected Diffusion-Planner trajectory. With no trajectory
  //    injected update() is a no-op (holds position), so this is safe before a trajectory arrives.
  vehicle_model_->setStateZInitialFrame(world_relative_position_.z());
  vehicle_model_->setGear(autoware_vehicle_msgs::msg::GearCommand::DRIVE);
  vehicle_model_->update(step_time);
  world_relative_position_.x() = vehicle_model_->getX();
  world_relative_position_.y() = vehicle_model_->getY();

  // 3. Reconstruct the map-frame pose (EgoEntitySimulation::getCurrentPose equivalent).
  const Eigen::Vector3d relative_position = initial_rotation_matrix_ * world_relative_position_;
  geometry_msgs::msg::Vector3 rpy;
  rpy.z = vehicle_model_->getYaw();
  geometry_msgs::msg::Pose updated_pose;
  updated_pose.position.x = initial_pose_.position.x + relative_position.x();
  updated_pose.position.y = initial_pose_.position.y + relative_position.y();
  updated_pose.position.z = initial_pose_.position.z + relative_position.z();
  updated_pose.orientation = initial_pose_.orientation * convertEulerAngleToQuaternion(rpy);

  // 4. Assemble the updated status and canonicalize against the current lanelets (z snaps to the
  //    lanelet spline on the next step's R^T roundtrip via world_relative_position_.z()).
  auto updated_status = static_cast<EntityStatus>(*status_);
  updated_status.time = current_time + step_time;
  updated_status.pose = updated_pose;
  updated_status.lanelet_pose_valid = false;
  updated_status.action_status.twist.linear.x = vehicle_model_->getVx();
  updated_status.action_status.twist.angular.z = vehicle_model_->getWz();
  updated_status.action_status.accel.linear.x = vehicle_model_->getAx();
  setStatus(updated_status, status_->getLaneletIds());

  EntityBase::onPostUpdate(current_time, step_time);
}

void EgoEntity::requestAcquirePosition(const CanonicalizedLaneletPose & lanelet_pose)
{
  traffic_simulator::RouteOption option;
  option.allow_goal_modification = get_parameter_or<bool>("allow_goal_modification", false);
  return requestAcquirePosition(lanelet_pose, option);
}

void EgoEntity::requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose)
{
  traffic_simulator::RouteOption option;
  option.allow_goal_modification = get_parameter_or<bool>("allow_goal_modification", false);
  return requestAcquirePosition(map_pose, option);
}

void EgoEntity::requestAcquirePosition(
  const CanonicalizedLaneletPose & lanelet_pose, const traffic_simulator::RouteOption & option)
{
  requestAssignRoute({lanelet_pose}, option);
}

void EgoEntity::requestAcquirePosition(
  const geometry_msgs::msg::Pose & map_pose, const traffic_simulator::RouteOption & option)
{
  requestAssignRoute({map_pose}, option);
}

void EgoEntity::requestAssignRoute(const std::vector<CanonicalizedLaneletPose> & route)
{
  traffic_simulator::RouteOption option;
  option.allow_goal_modification = get_parameter_or<bool>("allow_goal_modification", false);
  std::vector<geometry_msgs::msg::Pose> route_poses;
  for (const auto & lanelet_pose : route) {
    route_poses.push_back(static_cast<geometry_msgs::msg::Pose>(lanelet_pose));
  }
  return requestAssignRoute(route_poses, option);
}

void EgoEntity::requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> & route)
{
  traffic_simulator::RouteOption option;
  option.allow_goal_modification = get_parameter_or<bool>("allow_goal_modification", false);
  return requestAssignRoute(route, option);
}

void EgoEntity::requestAssignRoute(
  const std::vector<CanonicalizedLaneletPose> &, const traffic_simulator::RouteOption &)
{
  // Headless: goal/route handling is delegated to the route sidecar (R1c); no Autoware planner.
}

void EgoEntity::requestAssignRoute(
  const std::vector<geometry_msgs::msg::Pose> &, const traffic_simulator::RouteOption &)
{
  // Headless: goal/route handling is delegated to the route sidecar (R1c); no Autoware planner.
}

auto EgoEntity::isControlledBySimulator() const -> bool { return is_controlled_by_simulator_; }

auto EgoEntity::requestFollowTrajectory(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & parameter) -> void
{
  polyline_trajectory_ = parameter;
  VehicleEntity::requestFollowTrajectory(parameter);
  is_controlled_by_simulator_ = true;
}

auto EgoEntity::requestLaneChange(const lanelet::Id) -> void
{
  THROW_SEMANTIC_ERROR(
    "From scenario, a lane change was requested to Ego type entity ", std::quoted(name),
    " In general, such a request is an error, since Ego cars make autonomous decisions about "
    "everything but their destination");
}

auto EgoEntity::requestLaneChange(const traffic_simulator::lane_change::Parameter &) -> void
{
  THROW_SEMANTIC_ERROR(
    "From scenario, a lane change was requested to Ego type entity ", std::quoted(name),
    " In general, such a request is an error, since Ego cars make autonomous decisions about "
    "everything but their destination");
}

auto EgoEntity::requestSpeedChange(
  const double target_speed, const speed_change::Transition, const speed_change::Constraint,
  const bool) -> void
{
  requestSpeedChange(target_speed, false);
}

auto EgoEntity::requestSpeedChange(
  const speed_change::RelativeTargetSpeed &, const speed_change::Transition,
  const speed_change::Constraint, const bool) -> void
{
  THROW_SEMANTIC_ERROR(
    "The traffic_simulator's request to set speed to the Ego type entity is for initialization "
    "purposes only.");
}

auto EgoEntity::requestClearRoute() -> void {}

auto EgoEntity::requestReplanRoute(
  const std::vector<geometry_msgs::msg::PoseStamped> &, const bool) -> void
{
  // Headless: route replanning is delegated to the route sidecar (R1c); no Autoware planner.
}

auto EgoEntity::getDefaultDynamicConstraints() const
  -> const traffic_simulator_msgs::msg::DynamicConstraints &
{
  THROW_SEMANTIC_ERROR("getDefaultDynamicConstraints function does not support EgoEntity");
}

auto EgoEntity::setBehaviorParameter(
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter) -> void
{
  behavior_parameter_ = behavior_parameter;
}

auto EgoEntity::requestSpeedChange(double value, bool /* continuous */) -> void
{
  if (status_->getTime() > 0.0) {
    THROW_SEMANTIC_ERROR("You cannot set target speed to the ego vehicle after starting scenario.");
  } else {
    target_speed_ = value;
  }
}

auto EgoEntity::requestSpeedChange(
  const speed_change::RelativeTargetSpeed & /*target_speed*/, bool /*continuous*/) -> void
{
  THROW_SEMANTIC_ERROR(
    "The traffic_simulator's request to set speed to the Ego type entity is for initialization "
    "purposes only.");
}

auto EgoEntity::setVelocityLimit(double value) -> void
{
  behavior_parameter_.dynamic_constraints.max_speed = value;
}

auto EgoEntity::setMapPose(const geometry_msgs::msg::Pose & map_pose) -> void
{
  auto entity_status = static_cast<EntityStatus>(*status_);
  entity_status.pose = map_pose;
  entity_status.lanelet_pose_valid = false;
  status_->set(
    entity_status, helper::getUniqueValues(getRouteLanelets()),
    getDefaultMatchingDistanceForLaneletPoseCalculation());
}

auto EgoEntity::setDiffusionTrajectory(
  const rclcpp::Time & stamp, const autoware_planning_msgs::msg::Trajectory & trajectory) -> void
{
  if (vehicle_model_) {
    vehicle_model_->setTrajectory(stamp, trajectory);
  }
}

auto EgoEntity::setTurnIndicators(
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & command) -> void
{
  turn_indicators_command_ = command;
}
}  // namespace entity
}  // namespace traffic_simulator
