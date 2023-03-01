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

auto RoutePlanner::setWaypoints(const std::vector<CanonicalizedLaneletPoseType> & waypoints) -> void
{
  // Just setting waypoints to the queue, do not planning route.
  waypoint_queue_.clear();
  for (const auto & waypoint : waypoints) {
    waypoint_queue_.push_back(static_cast<traffic_simulator::LaneletPoseType>(waypoint));
  }
}

auto RoutePlanner::getRouteLanelets(
  const CanonicalizedLaneletPoseType & entity_lanelet_pose, double horizon)
  -> std::vector<std::int64_t>
{
  const auto lanelet_pose = static_cast<traffic_simulator::LaneletPoseType>(entity_lanelet_pose);
  // If the queue is not empty, calculating route from the entity_lanelet_pose to waypoint_queue_.front()
  if (!waypoint_queue_.empty()) {
    updateRoute(lanelet_pose);
  }
  // If the route from the entity_lanelet_pose to waypoint_queue_.front() was failed to calculate in updateRoute function,
  // use following lanelet as route.
  if (!route_) {
    return hdmap_utils_ptr_->getFollowingLanelets(lanelet_pose.lanelet_id, horizon, true);
  }
  // If the entity_lanelet_pose is in the lanelet id of the waypoint queue, cancel the target waypoint.
  cancelWaypoint(lanelet_pose);
  if (route_ && hdmap_utils_ptr_->isInRoute(lanelet_pose.lanelet_id, route_.get())) {
    return hdmap_utils_ptr_->getFollowingLanelets(
      lanelet_pose.lanelet_id, route_.get(), horizon, true);
  }
  return hdmap_utils_ptr_->getFollowingLanelets(lanelet_pose.lanelet_id, horizon, true);
}

void RoutePlanner::cancelRoute()
{
  waypoint_queue_.clear();
  route_ = boost::none;
}

void RoutePlanner::cancelWaypoint(const LaneletPoseType & entity_lanelet_pose)
{
  while (true) {
    if (waypoint_queue_.empty()) {
      cancelRoute();
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

std::vector<geometry_msgs::msg::Pose> RoutePlanner::getGoalPosesInWorldFrame() const
{
  std::vector<geometry_msgs::msg::Pose> ret;
  for (const auto & lanelet_pose : waypoint_queue_) {
    ret.emplace_back(hdmap_utils_ptr_->toMapPose(lanelet_pose).pose);
  }
  return ret;
}

std::vector<CanonicalizedLaneletPoseType> RoutePlanner::getGoalPoses() const
{
  std::vector<CanonicalizedLaneletPoseType> goal_poses;
  for (const auto & waypoint : waypoint_queue_) {
    goal_poses.emplace_back(CanonicalizedLaneletPoseType(waypoint, hdmap_utils_ptr_));
  }
  return goal_poses;
}

void RoutePlanner::updateRoute(const LaneletPoseType & entity_lanelet_pose)
{
  if (
    waypoint_queue_.front().lanelet_id == entity_lanelet_pose.lanelet_id &&
    waypoint_queue_.front().s <= entity_lanelet_pose.s) {
    cancelWaypoint(entity_lanelet_pose);
    if (waypoint_queue_.empty()) {
      cancelRoute();
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
