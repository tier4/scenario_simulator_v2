// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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
PedestrianEntity::PedestrianEntity(
  const std::string & name, const traffic_simulator_msgs::msg::PedestrianParameters & params,
  const std::string & plugin_name)
: EntityBase(params.pedestrian_category, name),
  parameters(params),
  plugin_name(plugin_name),
  loader_(pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase>(
    "traffic_simulator", "entity_behavior::BehaviorPluginBase")),
  behavior_plugin_ptr_(loader_.createSharedInstance(plugin_name))
{
  entity_type_.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  behavior_plugin_ptr_->configure(rclcpp::get_logger(name));
  behavior_plugin_ptr_->setPedestrianParameters(parameters);
  behavior_plugin_ptr_->setDebugMarker({});
  behavior_plugin_ptr_->setDriverModel(traffic_simulator_msgs::msg::DriverModel());
}

void PedestrianEntity::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array)
{
  const auto marker = behavior_plugin_ptr_->getDebugMarker();
  std::copy(marker.begin(), marker.end(), std::back_inserter(marker_array.markers));
}

void PedestrianEntity::requestAssignRoute(
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & waypoints)
{
  behavior_plugin_ptr_->setRequest("follow_lane");
  if (!status_) {
    return;
  }
  if (!status_->lanelet_pose_valid) {
    return;
  }
  route_planner_ptr_->getRouteLanelets(status_->lanelet_pose, waypoints);
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

void PedestrianEntity::requestWalkStraight() { behavior_plugin_ptr_->setRequest("walk_straight"); }

void PedestrianEntity::requestAcquirePosition(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
{
  behavior_plugin_ptr_->setRequest("follow_lane");
  if (!status_) {
    return;
  }
  if (!status_->lanelet_pose_valid) {
    return;
  }
  route_planner_ptr_->getRouteLanelets(status_->lanelet_pose, lanelet_pose);
}

void PedestrianEntity::requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose)
{
  const auto lanelet_pose = hdmap_utils_ptr_->toLaneletPose(map_pose, getBoundingBox(), true);
  if (lanelet_pose) {
    requestAcquirePosition(lanelet_pose.get());
  } else {
    THROW_SEMANTIC_ERROR("Goal of the pedestrian entity should be on lane.");
  }
}

void PedestrianEntity::requestSpeedChange(
  const double, const SpeedChangeTransition, const SpeedChangeConstraint, const bool)
{
  THROW_SEMANTIC_ERROR(
    "Currently, requestSpeedChange function does not works in pedestrian because users cannot "
    "specify acceleration and deceleration in pedestrian entity.");
}

void PedestrianEntity::cancelRequest()
{
  behavior_plugin_ptr_->setRequest("none");
  route_planner_ptr_->cancelGoal();
}

void PedestrianEntity::setTargetSpeed(double target_speed, bool continuous)
{
  target_speed_planner_.setTargetSpeed(target_speed, continuous);
}

void PedestrianEntity::setTargetSpeed(const RelativeTargetSpeed & target_speed, bool continuous)
{
  target_speed_planner_.setTargetSpeed(target_speed, continuous);
}

auto PedestrianEntity::getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel
{
  return behavior_plugin_ptr_->getDriverModel();
}

void PedestrianEntity::setDriverModel(const traffic_simulator_msgs::msg::DriverModel & driver_model)
{
  behavior_plugin_ptr_->setDriverModel(driver_model);
}

void PedestrianEntity::onUpdate(double current_time, double step_time)
{
  EntityBase::onUpdate(current_time, step_time);
  if (!status_) {
    return;
  }
  if (current_time < 0) {
    updateEntityStatusTimestamp(current_time);
  } else {
    behavior_plugin_ptr_->setOtherEntityStatus(other_status_);
    behavior_plugin_ptr_->setEntityTypeList(entity_type_list_);
    behavior_plugin_ptr_->setEntityStatus(status_.get());
    target_speed_planner_.update(status_->action_status.twist.linear.x, other_status_);
    behavior_plugin_ptr_->setTargetSpeed(target_speed_planner_.getTargetSpeed());
    if (status_->lanelet_pose_valid) {
      auto route = route_planner_ptr_->getRouteLanelets(status_->lanelet_pose);
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
    if (!status_) {
      linear_jerk_ = 0;
    } else {
      linear_jerk_ =
        (status_updated.action_status.accel.linear.x - status_->action_status.accel.linear.x) /
        step_time;
    }
    setStatus(status_updated);
    updateStandStillDuration(step_time);
  }
}
}  // namespace entity
}  // namespace traffic_simulator
