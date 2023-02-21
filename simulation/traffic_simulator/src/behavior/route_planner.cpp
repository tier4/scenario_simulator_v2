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

#include <traffic_simulator/behavior/route_planner.hpp>

namespace traffic_simulator
{
RoutePlanner::RoutePlanner(const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
: hdmap_utils_ptr_(hdmap_utils_ptr)
{
}

auto RoutePlanner::setWaypoints(
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & waypoints) -> void
{
  waypoint_queue_.clear();
  for (const auto & waypoint : waypoints) {
    waypoint_queue_.push_back(waypoint);
  }
}

auto RoutePlanner::getRouteLanelets(
  const traffic_simulator_msgs::msg::LaneletPose & entity_lanelet_pose, double horizon)
  -> std::vector<std::int64_t>
{
  if (!waypoint_queue_.empty()) {
    updateRoute(entity_lanelet_pose);
  }
  if (!route_) {
    return hdmap_utils_ptr_->getFollowingLanelets(entity_lanelet_pose.lanelet_id, horizon, true);
  }
  if (!waypoint_queue_.empty()) {
    if (waypoint_queue_.front().lanelet_id == entity_lanelet_pose.lanelet_id) {
      cancelGoal(entity_lanelet_pose);
    }
  } else {
    if (hdmap_utils_ptr_->isInRoute(entity_lanelet_pose.lanelet_id, route_.get())) {
      return hdmap_utils_ptr_->getFollowingLanelets(
        entity_lanelet_pose.lanelet_id, route_.get(), horizon, true);
    }
  }
  cancelGoal(entity_lanelet_pose);
  if (waypoint_queue_.empty()) {
    return hdmap_utils_ptr_->getFollowingLanelets(entity_lanelet_pose.lanelet_id, horizon, true);
  }
  if (not route_ or route_->empty()) {
    return hdmap_utils_ptr_->getFollowingLanelets(entity_lanelet_pose.lanelet_id, horizon, true);
  } else {
    return hdmap_utils_ptr_->getFollowingLanelets(
      entity_lanelet_pose.lanelet_id, route_.get(), horizon, true);
  }
}

void RoutePlanner::cancelGoal() { route_ = boost::none; }

void RoutePlanner::cancelGoal(const traffic_simulator_msgs::msg::LaneletPose & entity_lanelet_pose)
{
  while (true) {
    if (waypoint_queue_.empty()) {
      cancelGoal();
      break;
    }
    if (waypoint_queue_.front().lanelet_id == entity_lanelet_pose.lanelet_id) {
      waypoint_queue_.pop_front();
      continue;
    } else {
      break;
    }
  }
}

std::vector<geometry_msgs::msg::Pose> RoutePlanner::getGoalPosesInWorldFrame()
{
  std::vector<geometry_msgs::msg::Pose> ret;
  for (const auto & lanelet_pose : waypoint_queue_) {
    ret.emplace_back(hdmap_utils_ptr_->toMapPose(lanelet_pose).pose);
  }
  return ret;
}

std::vector<traffic_simulator_msgs::msg::LaneletPose> RoutePlanner::getGoalPoses()
{
  std::vector<traffic_simulator_msgs::msg::LaneletPose> goal_poses;
  std::copy(
    std::cbegin(waypoint_queue_), std::cend(waypoint_queue_), std::back_inserter(goal_poses));
  return goal_poses;
}

void RoutePlanner::updateRoute(const traffic_simulator_msgs::msg::LaneletPose & entity_lanelet_pose)
{
  if (
    waypoint_queue_.front().lanelet_id == entity_lanelet_pose.lanelet_id &&
    waypoint_queue_.front().s <= entity_lanelet_pose.s) {
    cancelGoal(entity_lanelet_pose);
    if (waypoint_queue_.empty()) {
      cancelGoal();
      return;
    }
  }
  if (!route_) {
    route_ = hdmap_utils_ptr_->getRoute(
      entity_lanelet_pose.lanelet_id, waypoint_queue_.front().lanelet_id);
    return;
  }
  if (hdmap_utils_ptr_->isInRoute(entity_lanelet_pose.lanelet_id, route_.get())) {
    return;
  } else {
    route_ = hdmap_utils_ptr_->getRoute(
      entity_lanelet_pose.lanelet_id, waypoint_queue_.front().lanelet_id);
    return;
  }
}
}  // namespace traffic_simulator
