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
#include <memory>
#include <string>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
VehicleEntity::VehicleEntity(
  const std::string & name, const CanonicalizedEntityStatus & entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters,
  const std::string & plugin_name)
: EntityBase(name, entity_status, hdmap_utils_ptr),
  vehicle_parameters(parameters),
  loader_(pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase>(
    "traffic_simulator", "entity_behavior::BehaviorPluginBase")),
  behavior_plugin_ptr_(loader_.createSharedInstance(plugin_name)),
  route_planner_(hdmap_utils_ptr_)
{
  behavior_plugin_ptr_->configure(rclcpp::get_logger(name));
  behavior_plugin_ptr_->setVehicleParameters(parameters);
  behavior_plugin_ptr_->setDebugMarker({});
  behavior_plugin_ptr_->setBehaviorParameter(traffic_simulator_msgs::msg::BehaviorParameter());
  behavior_plugin_ptr_->setHdMapUtils(hdmap_utils_ptr_);
  behavior_plugin_ptr_->setDefaultMatchingDistanceForLaneletPoseCalculation(
    getDefaultMatchingDistanceForLaneletPoseCalculation());
}

void VehicleEntity::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array)
{
  const auto marker = behavior_plugin_ptr_->getDebugMarker();
  std::copy(marker.begin(), marker.end(), std::back_inserter(marker_array.markers));
}

void VehicleEntity::cancelRequest()
{
  behavior_plugin_ptr_->setRequest(behavior::Request::NONE);
  route_planner_.cancelRoute();
}

auto VehicleEntity::getCurrentAction() const -> std::string
{
  return behavior_plugin_ptr_->getCurrentAction();
}

auto VehicleEntity::getDefaultDynamicConstraints() const
  -> const traffic_simulator_msgs::msg::DynamicConstraints &
{
  static auto default_dynamic_constraints = traffic_simulator_msgs::msg::DynamicConstraints();
  return default_dynamic_constraints;
}

auto VehicleEntity::getDefaultMatchingDistanceForLaneletPoseCalculation() const -> double
{
  return std::max(
           vehicle_parameters.axles.front_axle.track_width,
           vehicle_parameters.axles.rear_axle.track_width) *
           0.5 +
         1.0;
}

auto VehicleEntity::getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter
{
  return behavior_plugin_ptr_->getBehaviorParameter();
}

auto VehicleEntity::getEntityTypename() const -> const std::string &
{
  static const std::string result = "VehicleEntity";
  return result;
}

auto VehicleEntity::getGoalPoses() -> std::vector<CanonicalizedLaneletPose>
{
  return route_planner_.getGoalPoses();
}

auto VehicleEntity::getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return behavior_plugin_ptr_->getObstacle();
}

auto VehicleEntity::getRouteLanelets(double horizon) -> lanelet::Ids
{
  if (const auto canonicalized_lanelet_pose = status_->getCanonicalizedLaneletPose()) {
    return route_planner_.getRouteLanelets(canonicalized_lanelet_pose.value(), horizon);
  } else {
    return {};
  }
}

auto VehicleEntity::getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray
{
  try {
    return behavior_plugin_ptr_->getWaypoints();
  } catch (const std::runtime_error & e) {
    if (not status_->laneMatchingSucceed()) {
      THROW_SIMULATION_ERROR(
        "Failed to calculate waypoints in NPC logics, please check Entity : ", name,
        " is in a lane coordinate.");
    } else {
      THROW_SIMULATION_ERROR("Failed to calculate waypoint in NPC logics.");
    }
  }
}

auto VehicleEntity::onUpdate(const double current_time, const double step_time) -> void
{
  EntityBase::onUpdate(current_time, step_time);

  behavior_plugin_ptr_->setOtherEntityStatus(other_status_);
  behavior_plugin_ptr_->setCanonicalizedEntityStatus(status_);
  behavior_plugin_ptr_->setTargetSpeed(target_speed_);
  auto route_lanelets = getRouteLanelets();
  behavior_plugin_ptr_->setRouteLanelets(route_lanelets);

  // recalculate spline only when input data changes
  if (previous_route_lanelets_ != route_lanelets) {
    previous_route_lanelets_ = route_lanelets;
    try {
      spline_ = std::make_shared<math::geometry::CatmullRomSpline>(
        hdmap_utils_ptr_->getCenterPoints(route_lanelets));
    } catch (const common::scenario_simulator_exception::SemanticError & error) {
      // reset the ptr when spline cannot be calculated
      spline_.reset();
    }
  }
  behavior_plugin_ptr_->setReferenceTrajectory(spline_);
  /// @note CanonicalizedEntityStatus is updated here, it is not skipped even if isAtEndOfLanelets return true
  behavior_plugin_ptr_->update(current_time, step_time);
  if (const auto canonicalized_lanelet_pose = status_->getCanonicalizedLaneletPose()) {
    if (pose::isAtEndOfLanelets(canonicalized_lanelet_pose.value(), hdmap_utils_ptr_)) {
      stopAtCurrentPosition();
      updateStandStillDuration(step_time);
      updateTraveledDistance(step_time);
      return;
    }
  }
  updateStandStillDuration(step_time);
  updateTraveledDistance(step_time);

  EntityBase::onPostUpdate(current_time, step_time);
}

void VehicleEntity::requestAcquirePosition(const CanonicalizedLaneletPose & lanelet_pose)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  if (status_->laneMatchingSucceed()) {
    route_planner_.setWaypoints({lanelet_pose});
  }
  behavior_plugin_ptr_->setGoalPoses({static_cast<geometry_msgs::msg::Pose>(lanelet_pose)});
}

void VehicleEntity::requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  if (
    const auto canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
      map_pose, status_->getBoundingBox(), false,
      getDefaultMatchingDistanceForLaneletPoseCalculation(), hdmap_utils_ptr_)) {
    requestAcquirePosition(canonicalized_lanelet_pose.value());
  } else {
    THROW_SEMANTIC_ERROR("Goal of the vehicle entity should be on lane.");
  }
}

void VehicleEntity::requestAssignRoute(const std::vector<CanonicalizedLaneletPose> & waypoints)
{
  if (!laneMatchingSucceed()) {
    return;
  }
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  route_planner_.setWaypoints(waypoints);
  std::vector<geometry_msgs::msg::Pose> goal_poses;
  for (const auto & waypoint : waypoints) {
    goal_poses.emplace_back(static_cast<geometry_msgs::msg::Pose>(waypoint));
  }
  behavior_plugin_ptr_->setGoalPoses(goal_poses);
}

void VehicleEntity::requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  std::vector<CanonicalizedLaneletPose> route;
  for (const auto & waypoint : waypoints) {
    if (
      const auto canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
        waypoint, status_->getBoundingBox(), false,
        getDefaultMatchingDistanceForLaneletPoseCalculation(), hdmap_utils_ptr_)) {
      route.emplace_back(canonicalized_lanelet_pose.value());
    } else {
      THROW_SEMANTIC_ERROR("Waypoint of vehicle entity should be on lane.");
    }
  }
  requestAssignRoute(route);
}

auto VehicleEntity::requestFollowTrajectory(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & parameter) -> void
{
  behavior_plugin_ptr_->setPolylineTrajectory(parameter);
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_POLYLINE_TRAJECTORY);
  std::vector<CanonicalizedLaneletPose> waypoints;
  for (const auto & vertex : parameter->shape.vertices) {
    if (
      const auto canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
        vertex.position, status_->getBoundingBox(), false,
        getDefaultMatchingDistanceForLaneletPoseCalculation(), hdmap_utils_ptr_)) {
      waypoints.emplace_back(canonicalized_lanelet_pose.value());
    } else {
      /// @todo such a protection most likely makes sense, but test scenario
      /// RoutingAction.FollowTrajectoryAction-star has waypoints outside lanelet2
      // THROW_SEMANTIC_ERROR("FollowTrajectory waypoint should be on lane.");
    }
  }
  route_planner_.setWaypoints(waypoints);
}

void VehicleEntity::requestLaneChange(const lanelet::Id to_lanelet_id)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::LANE_CHANGE);
  const auto parameter = lane_change::Parameter(
    lane_change::AbsoluteTarget(to_lanelet_id), lane_change::TrajectoryShape::CUBIC,
    lane_change::Constraint());
  behavior_plugin_ptr_->setLaneChangeParameters(parameter);
}

void VehicleEntity::requestLaneChange(const traffic_simulator::lane_change::Parameter & parameter)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::LANE_CHANGE);
  behavior_plugin_ptr_->setLaneChangeParameters(parameter);
}

auto VehicleEntity::getMaxAcceleration() const -> double
{
  return std::clamp(
    getBehaviorParameter().dynamic_constraints.max_acceleration, 0.0,
    vehicle_parameters.performance.max_acceleration);
}

auto VehicleEntity::getMaxDeceleration() const -> double
{
  return std::clamp(
    getBehaviorParameter().dynamic_constraints.max_deceleration, 0.0,
    vehicle_parameters.performance.max_deceleration);
}

void VehicleEntity::setVelocityLimit(double linear_velocity)
{
  if (linear_velocity < 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration limit should be over zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_speed = linear_velocity;
  setBehaviorParameter(behavior_parameter);
}

void VehicleEntity::setAccelerationLimit(double acceleration)
{
  if (acceleration < 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration limit must be greater than or equal to zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_acceleration = acceleration;
  setBehaviorParameter(behavior_parameter);
}

void VehicleEntity::setAccelerationRateLimit(double acceleration_rate)
{
  if (acceleration_rate < 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration rate limit must be greater than or equal to zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_acceleration_rate = acceleration_rate;
  setBehaviorParameter(behavior_parameter);
}

void VehicleEntity::setDecelerationLimit(double deceleration)
{
  if (deceleration < 0.0) {
    THROW_SEMANTIC_ERROR("Deceleration limit must be greater than or equal to zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_deceleration = deceleration;
  setBehaviorParameter(behavior_parameter);
}

void VehicleEntity::setDecelerationRateLimit(double deceleration_rate)
{
  if (deceleration_rate < 0.0) {
    THROW_SEMANTIC_ERROR("Deceleration rate limit must be greater than or equal to zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_deceleration_rate = deceleration_rate;
  setBehaviorParameter(behavior_parameter);
}

void VehicleEntity::setBehaviorParameter(
  const traffic_simulator_msgs::msg::BehaviorParameter & parameter)
{
  behavior_plugin_ptr_->setBehaviorParameter(parameter);
}

void VehicleEntity::setTrafficLightManager(
  const std::shared_ptr<traffic_simulator::TrafficLightManager> & ptr)
{
  EntityBase::setTrafficLightManager(ptr);
  behavior_plugin_ptr_->setTrafficLightManager(traffic_light_manager_);
}

}  // namespace entity
}  // namespace traffic_simulator
