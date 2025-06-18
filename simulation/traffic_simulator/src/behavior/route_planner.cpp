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
RoutePlanner::RoutePlanner(
  const traffic_simulator::RoutingGraphType & routing_graph_type,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
: hdmap_utils_ptr_(hdmap_utils_ptr), routing_graph_type_(routing_graph_type)
{
}

auto RoutePlanner::setWaypoints(const std::vector<CanonicalizedLaneletPose> & waypoints) -> void
{
  // Just setting waypoints to the queue, do not planning route.
  waypoint_queue_.clear();
  whole_lanelet_ids_.clear();
  for (const auto & waypoint : waypoints) {
    waypoint_queue_.push_back(waypoint);
    if (whole_lanelet_ids_.empty()) {
      whole_lanelet_ids_.push_back(waypoint.getLaneletPose().lanelet_id);
    } else if (whole_lanelet_ids_.back() != waypoint.getLaneletPose().lanelet_id) {
      whole_lanelet_ids_.push_back(waypoint.getLaneletPose().lanelet_id);
    }
  }
}

auto RoutePlanner::getRouteLanelets(
  const CanonicalizedLaneletPose & entity_lanelet_pose, double horizon) -> lanelet::Ids
{
  const auto lanelet_pose = static_cast<LaneletPose>(entity_lanelet_pose);
  // If the queue is not empty, calculating route from the entity_lanelet_pose to waypoint_queue_.front()
  if (!waypoint_queue_.empty()) {
    updateRoute(entity_lanelet_pose);
  }
  // If the route from the entity_lanelet_pose to waypoint_queue_.front() was failed to calculate in updateRoute function,
  // use following lanelet as route.
  if (!route_) {
    return hdmap_utils_ptr_->getFollowingLanelets(
      lanelet_pose.lanelet_id, horizon, true, routing_graph_type_);
  }
  if (route_ && hdmap_utils_ptr_->isInRoute(lanelet_pose.lanelet_id, route_.value())) {
    return hdmap_utils_ptr_->getFollowingLanelets(
      lanelet_pose.lanelet_id, route_.value(), horizon, true, routing_graph_type_);
  }
  // If the entity_lanelet_pose is in the lanelet id of the waypoint queue, cancel the target waypoint.
  cancelWaypoint(entity_lanelet_pose);
  return hdmap_utils_ptr_->getFollowingLanelets(
    lanelet_pose.lanelet_id, horizon, true, routing_graph_type_);
}

void RoutePlanner::cancelRoute()
{
  waypoint_queue_.clear();
  route_ = std::nullopt;
}

void RoutePlanner::cancelWaypoint(const CanonicalizedLaneletPose & entity_lanelet_pose)
{
  while (true) {
    if (waypoint_queue_.empty()) {
      cancelRoute();
      break;
    }
    if (isSameLaneletId(waypoint_queue_.front(), entity_lanelet_pose)) {
      waypoint_queue_.pop_front();
      continue;
    } else {
      break;
    }
  }
}

auto RoutePlanner::getGoalPosesInWorldFrame() const -> std::vector<geometry_msgs::msg::Pose>
{
  std::vector<geometry_msgs::msg::Pose> ret;
  for (const auto & lanelet_pose : waypoint_queue_) {
    ret.emplace_back(static_cast<geometry_msgs::msg::Pose>(lanelet_pose));
  }
  return ret;
}

auto RoutePlanner::getGoalPoses() const -> std::vector<CanonicalizedLaneletPose>
{
  std::vector<CanonicalizedLaneletPose> goal_poses;
  for (const auto & waypoint : waypoint_queue_) {
    goal_poses.emplace_back(waypoint);
  }
  return goal_poses;
}

auto RoutePlanner::updateRoute(const CanonicalizedLaneletPose & entity_lanelet_pose) -> void
{
  if (waypoint_queue_.front() <= entity_lanelet_pose) {
    cancelWaypoint(entity_lanelet_pose);
    if (waypoint_queue_.empty()) {
      cancelRoute();
      return;
    }
  }
  const auto lanelet_pose = static_cast<LaneletPose>(entity_lanelet_pose);
  static auto routing_configuration = [this]() {
    RoutingConfiguration config;
    config.routing_graph_type = routing_graph_type_;
    config.allow_lane_change = false;
    return config;
  }();
  if (!route_) {
    route_ = hdmap_utils_ptr_->getRoute(
      lanelet_pose.lanelet_id, static_cast<LaneletPose>(waypoint_queue_.front()).lanelet_id,
      routing_configuration);
    return;
  }
  if (hdmap_utils_ptr_->isInRoute(lanelet_pose.lanelet_id, route_.value())) {
    return;
  } else {
    route_ = hdmap_utils_ptr_->getRoute(
      lanelet_pose.lanelet_id, static_cast<LaneletPose>(waypoint_queue_.front()).lanelet_id,
      routing_configuration);
    return;
  }
}
}  // namespace traffic_simulator
