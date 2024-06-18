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
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
PedestrianEntity::PedestrianEntity(
  const std::string & name, const CanonicalizedEntityStatus & entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
  const traffic_simulator_msgs::msg::PedestrianParameters & parameters,
  const std::string & plugin_name)
: EntityBase(name, entity_status, hdmap_utils_ptr),
  plugin_name(plugin_name),
  pedestrian_parameters(parameters),
  loader_(pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase>(
    "traffic_simulator", "entity_behavior::BehaviorPluginBase")),
  behavior_plugin_ptr_(loader_.createSharedInstance(plugin_name)),
  route_planner_(hdmap_utils_ptr_)
{
  behavior_plugin_ptr_->configure(rclcpp::get_logger(name));
  behavior_plugin_ptr_->setPedestrianParameters(parameters);
  behavior_plugin_ptr_->setDebugMarker({});
  behavior_plugin_ptr_->setBehaviorParameter(traffic_simulator_msgs::msg::BehaviorParameter());
  behavior_plugin_ptr_->setHdMapUtils(hdmap_utils_ptr_);
  behavior_plugin_ptr_->setDefaultMatchingDistanceForLaneletPoseCalculation(
    getDefaultMatchingDistanceForLaneletPoseCalculation());
}

void PedestrianEntity::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array)
{
  const auto marker = behavior_plugin_ptr_->getDebugMarker();
  std::copy(marker.begin(), marker.end(), std::back_inserter(marker_array.markers));
}

void PedestrianEntity::requestAssignRoute(const std::vector<CanonicalizedLaneletPose> & waypoints)
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

void PedestrianEntity::requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  std::vector<CanonicalizedLaneletPose> route;
  for (const auto & waypoint : waypoints) {
    const auto lanelet_waypoint = hdmap_utils_ptr_->toLaneletPose(waypoint, getBoundingBox(), true);
    if (lanelet_waypoint) {
      route.emplace_back(CanonicalizedLaneletPose(lanelet_waypoint.value(), hdmap_utils_ptr_));
    } else {
      THROW_SEMANTIC_ERROR("Waypoint of pedestrian entity should be on lane.");
    }
  }
  requestAssignRoute(route);
}

auto PedestrianEntity::requestFollowTrajectory(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & parameter) -> void
{
  behavior_plugin_ptr_->setPolylineTrajectory(parameter);
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_POLYLINE_TRAJECTORY);
}

std::string PedestrianEntity::getCurrentAction() const
{
  if (!npc_logic_started_) {
    return "waiting";
  }
  return behavior_plugin_ptr_->getCurrentAction();
}

auto PedestrianEntity::getDefaultDynamicConstraints() const
  -> const traffic_simulator_msgs::msg::DynamicConstraints &
{
  static auto default_dynamic_constraints = traffic_simulator_msgs::msg::DynamicConstraints();
  default_dynamic_constraints.max_acceleration = 1.0;
  default_dynamic_constraints.max_acceleration_rate = 1.0;
  default_dynamic_constraints.max_deceleration = 1.0;
  default_dynamic_constraints.max_deceleration_rate = 1.0;
  return default_dynamic_constraints;
}

auto PedestrianEntity::getRouteLanelets(double horizon) -> lanelet::Ids
{
  if (status_.laneMatchingSucceed()) {
    return route_planner_.getRouteLanelets(
      CanonicalizedLaneletPose(status_.getLaneletPose(), hdmap_utils_ptr_), horizon);
  } else {
    return {};
  }
}

auto PedestrianEntity::getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return std::nullopt;
}

auto PedestrianEntity::getGoalPoses() -> std::vector<CanonicalizedLaneletPose>
{
  return route_planner_.getGoalPoses();
}

const traffic_simulator_msgs::msg::WaypointsArray PedestrianEntity::getWaypoints()
{
  return traffic_simulator_msgs::msg::WaypointsArray();
}

void PedestrianEntity::requestWalkStraight()
{
  behavior_plugin_ptr_->setRequest(behavior::Request::WALK_STRAIGHT);
}

void PedestrianEntity::requestAcquirePosition(const CanonicalizedLaneletPose & lanelet_pose)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  if (status_.laneMatchingSucceed()) {
    route_planner_.setWaypoints({lanelet_pose});
  }
  behavior_plugin_ptr_->setGoalPoses({static_cast<geometry_msgs::msg::Pose>(lanelet_pose)});
}

void PedestrianEntity::requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  if (const auto lanelet_pose = hdmap_utils_ptr_->toLaneletPose(map_pose, getBoundingBox(), true);
      lanelet_pose) {
    requestAcquirePosition(CanonicalizedLaneletPose(lanelet_pose.value(), hdmap_utils_ptr_));
  } else {
    THROW_SEMANTIC_ERROR("Goal of the pedestrian entity should be on lane.");
  }
}

void PedestrianEntity::cancelRequest()
{
  behavior_plugin_ptr_->setRequest(behavior::Request::NONE);
  route_planner_.cancelRoute();
}

auto PedestrianEntity::getEntityType() const -> const traffic_simulator_msgs::msg::EntityType &
{
  static traffic_simulator_msgs::msg::EntityType type;
  type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  return type;
}

auto PedestrianEntity::getEntityTypename() const -> const std::string &
{
  static const std::string result = "PedestrianEntity";
  return result;
}

void PedestrianEntity::setTrafficLightManager(
  const std::shared_ptr<traffic_simulator::TrafficLightManager> & ptr)
{
  EntityBase::setTrafficLightManager(ptr);
  behavior_plugin_ptr_->setTrafficLightManager(traffic_light_manager_);
}

auto PedestrianEntity::getBehaviorParameter() const
  -> traffic_simulator_msgs::msg::BehaviorParameter
{
  return behavior_plugin_ptr_->getBehaviorParameter();
}

void PedestrianEntity::setBehaviorParameter(
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter)
{
  behavior_plugin_ptr_->setBehaviorParameter(behavior_parameter);
}

void PedestrianEntity::setVelocityLimit(double linear_velocity)
{
  if (linear_velocity < 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration limit should be over zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_speed = linear_velocity;
  setBehaviorParameter(behavior_parameter);
}

void PedestrianEntity::setAccelerationLimit(double acceleration)
{
  if (acceleration < 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration limit should be over zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_acceleration = acceleration;
  setBehaviorParameter(behavior_parameter);
}

void PedestrianEntity::setAccelerationRateLimit(double acceleration_rate)
{
  if (acceleration_rate < 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration rate limit should be over zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_acceleration_rate = acceleration_rate;
  setBehaviorParameter(behavior_parameter);
}

void PedestrianEntity::setDecelerationLimit(double deceleration)
{
  if (deceleration < 0.0) {
    THROW_SEMANTIC_ERROR("Deceleration limit should be over zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_deceleration = deceleration;
  setBehaviorParameter(behavior_parameter);
}

void PedestrianEntity::setDecelerationRateLimit(double deceleration_rate)
{
  if (deceleration_rate < 0.0) {
    THROW_SEMANTIC_ERROR("Deceleration rate limit should be over zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_deceleration_rate = deceleration_rate;
  setBehaviorParameter(behavior_parameter);
}

auto PedestrianEntity::fillLaneletPose(CanonicalizedEntityStatus & status) -> void
{
  EntityBase::fillLaneletPose(status, true);
}

void PedestrianEntity::onUpdate(double current_time, double step_time)
{
  EntityBase::onUpdate(current_time, step_time);
  if (npc_logic_started_) {
    behavior_plugin_ptr_->setOtherEntityStatus(other_status_);
    behavior_plugin_ptr_->setEntityStatus(
      std::make_shared<traffic_simulator::CanonicalizedEntityStatus>(status_));
    behavior_plugin_ptr_->setTargetSpeed(target_speed_);
    behavior_plugin_ptr_->setRouteLanelets(getRouteLanelets());
    behavior_plugin_ptr_->update(current_time, step_time);
    auto status_updated = behavior_plugin_ptr_->getUpdatedStatus();
    if (status_updated->laneMatchingSucceed()) {
      const auto lanelet_pose = status_updated->getLaneletPose();
      if (
        hdmap_utils_ptr_->getFollowingLanelets(lanelet_pose.lanelet_id).size() == 1 &&
        hdmap_utils_ptr_->getLaneletLength(lanelet_pose.lanelet_id) <= lanelet_pose.s) {
        stopAtCurrentPosition();
        updateStandStillDuration(step_time);
        updateTraveledDistance(step_time);
        return;
      }
    }
    setStatus(*status_updated);
    updateStandStillDuration(step_time);
    updateTraveledDistance(step_time);
  } else {
    updateEntityStatusTimestamp(current_time);
  }
  EntityBase::onPostUpdate(current_time, step_time);
}
}  // namespace entity
}  // namespace traffic_simulator
