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

#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <memory>
#include <string>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
VehicleEntity::VehicleEntity(
  const std::string & name, const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters,
  const std::string & plugin_name)
: EntityBase(name, entity_status),
  loader_(pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase>(
    "traffic_simulator", "entity_behavior::BehaviorPluginBase")),
  behavior_plugin_ptr_(loader_.createSharedInstance(plugin_name))
{
  behavior_plugin_ptr_->configure(rclcpp::get_logger(name));
  behavior_plugin_ptr_->setVehicleParameters(parameters);
  behavior_plugin_ptr_->setDebugMarker({});
  behavior_plugin_ptr_->setBehaviorParameter(traffic_simulator_msgs::msg::BehaviorParameter());
}

void VehicleEntity::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array)
{
  const auto marker = behavior_plugin_ptr_->getDebugMarker();
  std::copy(marker.begin(), marker.end(), std::back_inserter(marker_array.markers));
}

void VehicleEntity::cancelRequest()
{
  behavior_plugin_ptr_->setRequest(behavior::Request::NONE);
  route_planner_ptr_->cancelGoal();
}

auto VehicleEntity::getCurrentAction() const -> std::string
{
  if (not npc_logic_started_) {
    return "waiting";
  } else {
    return behavior_plugin_ptr_->getCurrentAction();
  }
}

auto VehicleEntity::getDefaultDynamicConstraints() const
  -> const traffic_simulator_msgs::msg::DynamicConstraints &
{
  static auto default_dynamic_constraints = traffic_simulator_msgs::msg::DynamicConstraints();
  return default_dynamic_constraints;
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

auto VehicleEntity::getGoalPoses() -> std::vector<traffic_simulator_msgs::msg::LaneletPose>
{
  return route_planner_ptr_->getGoalPoses();
}

auto VehicleEntity::getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return behavior_plugin_ptr_->getObstacle();
}

auto VehicleEntity::getRouteLanelets(double horizon) const -> std::vector<std::int64_t>
{
  if (status_.lanelet_pose_valid) {
    return route_planner_ptr_->getRouteLanelets(status_.lanelet_pose, horizon);
  } else {
    return {};
  }
}

auto VehicleEntity::getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray
{
  if (!npc_logic_started_) {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
  try {
    return behavior_plugin_ptr_->getWaypoints();
  } catch (const std::runtime_error & e) {
    if (not status_.lanelet_pose_valid) {
      THROW_SIMULATION_ERROR(
        "Failed to calculate waypoints in NPC logics, please check Entity : ", name,
        " is in a lane coordinate.");
    } else {
      THROW_SIMULATION_ERROR("Failed to calculate waypoint in NPC logics.");
    }
  }
}

void VehicleEntity::onUpdate(double current_time, double step_time)
{
  EntityBase::onUpdate(current_time, step_time);
  if (npc_logic_started_) {
    behavior_plugin_ptr_->setOtherEntityStatus(other_status_);
    behavior_plugin_ptr_->setEntityTypeList(entity_type_list_);
    behavior_plugin_ptr_->setEntityStatus(status_);
    behavior_plugin_ptr_->setTargetSpeed(target_speed_);

    std::vector<std::int64_t> route_lanelets = {};
    if (status_.lanelet_pose_valid) {
      route_lanelets = route_planner_ptr_->getRouteLanelets(status_.lanelet_pose);
    }
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
    behavior_plugin_ptr_->update(current_time, step_time);
    auto status_updated = behavior_plugin_ptr_->getUpdatedStatus();
    if (status_updated.lanelet_pose_valid) {
      auto following_lanelets =
        hdmap_utils_ptr_->getFollowingLanelets(status_updated.lanelet_pose.lanelet_id);
      auto l = hdmap_utils_ptr_->getLaneletLength(status_updated.lanelet_pose.lanelet_id);
      if (following_lanelets.size() == 1 && l <= status_updated.lanelet_pose.s) {
        stopAtEndOfRoad();
        return;
      }
    }

    setStatus(status_updated);
    updateStandStillDuration(step_time);
    updateTraveledDistance(step_time);
  } else {
    updateEntityStatusTimestamp(current_time);
  }
  EntityBase::onPostUpdate(current_time, step_time);
}

void VehicleEntity::requestAcquirePosition(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
{
  if (status_.lanelet_pose_valid) {
    route_planner_ptr_->getRouteLanelets(status_.lanelet_pose, lanelet_pose);
  }
  behavior_plugin_ptr_->setGoalPoses(
    {hdmap_utils_ptr_->toMapPose(lanelet_pose.lanelet_id, lanelet_pose.s, lanelet_pose.offset)
       .pose});
}

void VehicleEntity::requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose)
{
  if (const auto lanelet_pose =
        hdmap_utils_ptr_->toLaneletPose(map_pose, getStatus().bounding_box, false);
      lanelet_pose) {
    requestAcquirePosition(lanelet_pose.value());
  } else {
    THROW_SEMANTIC_ERROR("Goal of the vehicle entity should be on lane.");
  }
}

void VehicleEntity::requestAssignRoute(
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & waypoints)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  if (!status_.lanelet_pose_valid) {
    return;
  }
  std::vector<geometry_msgs::msg::Pose> goal_poses;
  for (const auto & point : hdmap_utils_ptr_->getCenterPoints(
         route_planner_ptr_->getRouteLanelets(status_.lanelet_pose, waypoints))) {
    geometry_msgs::msg::Pose pose;
    pose.position = point;
    if (
      const auto lanelet_pose =
        hdmap_utils_ptr_->toLaneletPose(pose, getStatus().bounding_box, true)) {
      if (!lanelet_pose.has_value()) THROW_SEMANTIC_ERROR("Optional lanelet_pose has no value!");
      const auto map_pose_stamped = hdmap_utils_ptr_->toMapPose(
        lanelet_pose.value().lanelet_id, lanelet_pose.value().s, lanelet_pose.value().offset);
      goal_poses.emplace_back(map_pose_stamped.pose);
    }
  }
  behavior_plugin_ptr_->setGoalPoses(goal_poses);
}

void VehicleEntity::requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  std::vector<traffic_simulator_msgs::msg::LaneletPose> route;
  for (const auto & waypoint : waypoints) {
    if (const auto lanelet_waypoint =
          hdmap_utils_ptr_->toLaneletPose(waypoint, getStatus().bounding_box, false);
        lanelet_waypoint) {
      route.emplace_back(lanelet_waypoint.value());
    } else {
      THROW_SEMANTIC_ERROR("Waypoint of pedestrian entity should be on lane.");
    }
  }
  requestAssignRoute(route);
}

auto VehicleEntity::requestFollowTrajectory(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & parameter) -> void
{
  behavior_plugin_ptr_->setPolylineTrajectory(parameter);
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_POLYLINE_TRAJECTORY);
}

void VehicleEntity::requestLaneChange(const std::int64_t to_lanelet_id)
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

void VehicleEntity::setAccelerationLimit(double acceleration)
{
  if (acceleration <= 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration limit must be greater than or equal to zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_acceleration = acceleration;
  setBehaviorParameter(behavior_parameter);
}

void VehicleEntity::setAccelerationRateLimit(double acceleration_rate)
{
  if (acceleration_rate <= 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration rate limit must be greater than or equal to zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_acceleration_rate = acceleration_rate;
  setBehaviorParameter(behavior_parameter);
}

void VehicleEntity::setDecelerationLimit(double deceleration)
{
  if (deceleration <= 0.0) {
    THROW_SEMANTIC_ERROR("Deceleration limit must be greater than or equal to zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_deceleration = deceleration;
  setBehaviorParameter(behavior_parameter);
}

void VehicleEntity::setDecelerationRateLimit(double deceleration_rate)
{
  if (deceleration_rate <= 0.0) {
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

void VehicleEntity::setHdMapUtils(const std::shared_ptr<hdmap_utils::HdMapUtils> & ptr)
{
  EntityBase::setHdMapUtils(ptr);
  route_planner_ptr_ = std::make_shared<traffic_simulator::RoutePlanner>(ptr);
  behavior_plugin_ptr_->setHdMapUtils(hdmap_utils_ptr_);
}

void VehicleEntity::setTrafficLightManager(
  const std::shared_ptr<traffic_simulator::TrafficLightManager> & ptr)
{
  EntityBase::setTrafficLightManager(ptr);
  behavior_plugin_ptr_->setTrafficLightManager(traffic_light_manager_);
}

auto VehicleEntity::fillLaneletPose(traffic_simulator_msgs::msg::EntityStatus & status) const
  -> void
{
  EntityBase::fillLaneletPose(status, false);
}

}  // namespace entity
}  // namespace traffic_simulator
