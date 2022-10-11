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

#include <boost/algorithm/clamp.hpp>
#include <memory>
#include <string>
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
void PedestrianEntity::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array)
{
  const auto marker = behavior_plugin_ptr_->getDebugMarker();
  std::copy(marker.begin(), marker.end(), std::back_inserter(marker_array.markers));
}

auto PedestrianEntity::getEntityType() const -> const traffic_simulator_msgs::msg::EntityType &
{
  static const auto entity_type = []() {
    traffic_simulator_msgs::msg::EntityType entity_type;
    entity_type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
    return entity_type;
  }();

  return entity_type;
}

void PedestrianEntity::requestAssignRoute(
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & waypoints)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::FOLLOW_LANE);
  if (status_.lanelet_pose_valid) {
    route_planner_ptr_->getRouteLanelets(status_.lanelet_pose, waypoints);
  }
}

void PedestrianEntity::requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  std::vector<traffic_simulator_msgs::msg::LaneletPose> route;
  for (const auto & waypoint : waypoints) {
    const auto lanelet_waypoint = hdmap_utils_ptr_->toLaneletPose(waypoint, getBoundingBox(), true);
    if (lanelet_waypoint) {
      route.emplace_back(lanelet_waypoint.get());
    } else {
      THROW_SEMANTIC_ERROR("Waypoint of pedestrian entity should be on lane.");
    }
  }
  requestAssignRoute(route);
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
}

void PedestrianEntity::requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose)
{
  if (const auto lanelet_pose = hdmap_utils_ptr_->toLaneletPose(map_pose, getBoundingBox(), true);
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

auto PedestrianEntity::getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel
{
  return behavior_plugin_ptr_->getDriverModel();
}

void PedestrianEntity::setDriverModel(const traffic_simulator_msgs::msg::DriverModel & driver_model)
{
  behavior_plugin_ptr_->setDriverModel(driver_model);
}

void PedestrianEntity::setAccelerationLimit(double acceleration)
{
  if (acceleration <= 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration limit should be over zero.");
  }
  auto driver_model = getDriverModel();
  driver_model.acceleration = acceleration;
  setDriverModel(driver_model);
}

void PedestrianEntity::setDecelerationLimit(double deceleration)
{
  if (deceleration <= 0.0) {
    THROW_SEMANTIC_ERROR("Deceleration limit should be over zero.");
  }
  auto driver_model = getDriverModel();
  driver_model.deceleration = deceleration;
  setDriverModel(driver_model);
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
    linear_jerk_ =
      (status_updated.action_status.accel.linear.x - status_.action_status.accel.linear.x) /
      step_time;
    setStatus(status_updated);
    updateStandStillDuration(step_time);
  } else {
    updateEntityStatusTimestamp(current_time);
  }
}
}  // namespace entity
}  // namespace traffic_simulator
