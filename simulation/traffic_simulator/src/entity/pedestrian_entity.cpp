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
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
PedestrianEntity::PedestrianEntity(
  const std::string & name, const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const traffic_simulator_msgs::msg::PedestrianParameters & parameters,
  const std::string & plugin_name)
: EntityBase(name, entity_status),
  plugin_name(plugin_name),
  loader_(pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase>(
    "traffic_simulator", "entity_behavior::BehaviorPluginBase")),
  behavior_plugin_ptr_(loader_.createSharedInstance(plugin_name))
{
  behavior_plugin_ptr_->configure(rclcpp::get_logger(name));
  behavior_plugin_ptr_->setPedestrianParameters(parameters);
  behavior_plugin_ptr_->setDebugMarker({});
  behavior_plugin_ptr_->setBehaviorParameter(traffic_simulator_msgs::msg::BehaviorParameter());
}

void PedestrianEntity::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array)
{
  const auto marker = behavior_plugin_ptr_->getDebugMarker();
  std::copy(marker.begin(), marker.end(), std::back_inserter(marker_array.markers));
}

void PedestrianEntity::requestAssignRoute(
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & waypoints)
{
  if (!status_.lanelet_pose_valid) {
    return;
  }
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  std::vector<geometry_msgs::msg::Pose> goal_poses;
  for (const auto & point : hdmap_utils_ptr_->getCenterPoints(
         route_planner_ptr_->getRouteLanelets(status_.lanelet_pose, waypoints))) {
    geometry_msgs::msg::Pose pose;
    pose.position = point;
    if (
      const auto lanelet_pose =
        hdmap_utils_ptr_->toLaneletPose(pose, getStatus().bounding_box, true)) {
      const auto map_pose_stamped = hdmap_utils_ptr_->toMapPose(
        lanelet_pose.get().lanelet_id, lanelet_pose.get().s, lanelet_pose.get().offset);
      goal_poses.emplace_back(map_pose_stamped.pose);
    }
  }
  behavior_plugin_ptr_->setGoalPoses(goal_poses);
}

void PedestrianEntity::requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  std::vector<traffic_simulator_msgs::msg::LaneletPose> route;
  for (const auto & waypoint : waypoints) {
    const auto lanelet_waypoint =
      hdmap_utils_ptr_->toLaneletPose(waypoint, getStatus().bounding_box, true);
    if (lanelet_waypoint) {
      route.emplace_back(lanelet_waypoint.get());
    } else {
      THROW_SEMANTIC_ERROR("Waypoint of pedestrian entity should be on lane.");
    }
  }
  requestAssignRoute(route);
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

std::vector<std::int64_t> PedestrianEntity::getRouteLanelets(double horizon)
{
  if (status_.lanelet_pose_valid) {
    return route_planner_ptr_->getRouteLanelets(status_.lanelet_pose, horizon);
  } else {
    return {};
  }
}

auto PedestrianEntity::getObstacle() -> boost::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return boost::none;
}

auto PedestrianEntity::getGoalPoses() -> std::vector<traffic_simulator_msgs::msg::LaneletPose>
{
  return route_planner_ptr_->getGoalPoses();
}

const traffic_simulator_msgs::msg::WaypointsArray PedestrianEntity::getWaypoints()
{
  return traffic_simulator_msgs::msg::WaypointsArray();
}

void PedestrianEntity::requestWalkStraight()
{
  behavior_plugin_ptr_->setRequest(behavior::Request::WALK_STRAIGHT);
}

void PedestrianEntity::requestAcquirePosition(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  if (status_.lanelet_pose_valid) {
    route_planner_ptr_->getRouteLanelets(status_.lanelet_pose, lanelet_pose);
  }
  behavior_plugin_ptr_->setGoalPoses(
    {hdmap_utils_ptr_->toMapPose(lanelet_pose.lanelet_id, lanelet_pose.s, lanelet_pose.offset)
       .pose});
}

void PedestrianEntity::requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose)
{
  if (const auto lanelet_pose =
        hdmap_utils_ptr_->toLaneletPose(map_pose, getStatus().bounding_box, true);
      lanelet_pose) {
    requestAcquirePosition(lanelet_pose.get());
  } else {
    THROW_SEMANTIC_ERROR("Goal of the pedestrian entity should be on lane.");
  }
}

void PedestrianEntity::cancelRequest()
{
  behavior_plugin_ptr_->setRequest(behavior::Request::NONE);
  route_planner_ptr_->cancelGoal();
}

auto PedestrianEntity::getEntityTypename() const -> const std::string &
{
  static const std::string result = "PedestrianEntity";
  return result;
}

void PedestrianEntity::setHdMapUtils(const std::shared_ptr<hdmap_utils::HdMapUtils> & ptr)
{
  EntityBase::setHdMapUtils(ptr);
  route_planner_ptr_ = std::make_shared<traffic_simulator::RoutePlanner>(ptr);
  behavior_plugin_ptr_->setHdMapUtils(hdmap_utils_ptr_);
}

void PedestrianEntity::setTrafficLightManager(
  const std::shared_ptr<traffic_simulator::TrafficLightManagerBase> & ptr)
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

void PedestrianEntity::setAccelerationLimit(double acceleration)
{
  if (acceleration <= 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration limit should be over zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_acceleration = acceleration;
  setBehaviorParameter(behavior_parameter);
}

void PedestrianEntity::setAccelerationRateLimit(double acceleration_rate)
{
  if (acceleration_rate <= 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration rate limit should be over zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_acceleration_rate = acceleration_rate;
  setBehaviorParameter(behavior_parameter);
}

void PedestrianEntity::setDecelerationLimit(double deceleration)
{
  if (deceleration <= 0.0) {
    THROW_SEMANTIC_ERROR("Deceleration limit should be over zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_deceleration = deceleration;
  setBehaviorParameter(behavior_parameter);
}

void PedestrianEntity::setDecelerationRateLimit(double deceleration_rate)
{
  if (deceleration_rate <= 0.0) {
    THROW_SEMANTIC_ERROR("Deceleration rate limit should be over zero.");
  }
  auto behavior_parameter = getBehaviorParameter();
  behavior_parameter.dynamic_constraints.max_deceleration_rate = deceleration_rate;
  setBehaviorParameter(behavior_parameter);
}

void PedestrianEntity::onUpdate(double current_time, double step_time)
{
  EntityBase::onUpdate(current_time, step_time);
  if (npc_logic_started_) {
    behavior_plugin_ptr_->setOtherEntityStatus(other_status_);
    behavior_plugin_ptr_->setEntityTypeList(entity_type_list_);
    behavior_plugin_ptr_->setEntityStatus(status_);
    behavior_plugin_ptr_->setTargetSpeed(target_speed_);
    if (status_.lanelet_pose_valid) {
      auto route = route_planner_ptr_->getRouteLanelets(status_.lanelet_pose);
      behavior_plugin_ptr_->setRouteLanelets(route);
    } else {
      std::vector<std::int64_t> empty = {};
      behavior_plugin_ptr_->setRouteLanelets(empty);
    }
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
}  // namespace entity
}  // namespace traffic_simulator
