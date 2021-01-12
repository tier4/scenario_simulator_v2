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

#include <simulation_api/behavior/vehicle/behavior_tree.hpp>
#include <simulation_api/behavior/vehicle/lane_change_action.hpp>
#include <simulation_api/entity/vehicle_parameter.hpp>
#include <simulation_api/math/catmull_rom_spline.hpp>

#include <string>
#include <memory>
#include <algorithm>
#include <vector>

namespace entity_behavior
{
namespace vehicle
{
LaneChangeAction::LaneChangeAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config) {}

const boost::optional<openscenario_msgs::msg::Obstacle> LaneChangeAction::calculateObstacle(
  const openscenario_msgs::msg::WaypointsArray & waypoints)
{
  return boost::none;
}

const openscenario_msgs::msg::WaypointsArray LaneChangeAction::calculateWaypoints()
{
  if (!curve_) {
    throw BehaviorTreeRuntimeError("curve is null");
  }
  if (!to_lanelet_id_) {
    throw BehaviorTreeRuntimeError("to lanelet id is null");
  }
  if (entity_status.action_status.twist.linear.x >= 0) {
    openscenario_msgs::msg::WaypointsArray waypoints;
    double horizon =
      boost::algorithm::clamp(entity_status.action_status.twist.linear.x * 5, 20, 50);
    auto following_lanelets = hdmap_utils->getFollowingLanelets(
      to_lanelet_id_.get(), 0);
    double l = curve_->getLength();
    double rest_s = current_s_ + horizon - l;
    if (rest_s < 0) {
      const auto curve_waypoints =
        curve_->getTrajectory(current_s_, current_s_ + horizon, 1.0, true);
      waypoints.waypoints = curve_waypoints;
    } else {
      std::vector<geometry_msgs::msg::Point> center_points = hdmap_utils->getCenterPoints(
        following_lanelets);
      simulation_api::math::CatmullRomSpline spline(center_points);
      const auto straight_waypoints = spline.getTrajectory(target_s_, target_s_ + rest_s, 1.0);
      waypoints.waypoints = straight_waypoints;
      const auto curve_waypoints = curve_->getTrajectory(current_s_, l, 1.0, true);
      waypoints.waypoints = curve_waypoints;
      std::copy(
        straight_waypoints.begin(), straight_waypoints.end(),
        std::back_inserter(waypoints.waypoints));
    }
    return waypoints;
  } else {
    return openscenario_msgs::msg::WaypointsArray();
  }
}

void LaneChangeAction::getBlackBoardValues()
{
  VehicleActionNode::getBlackBoardValues();
  std::int64_t to_lanelet_id;
  if (!getInput<std::int64_t>("to_lanelet_id", to_lanelet_id)) {
    to_lanelet_id_ = boost::none;
  } else {
    to_lanelet_id_ = to_lanelet_id;
  }
}

BT::NodeStatus LaneChangeAction::tick()
{
  getBlackBoardValues();
  if (request != "lane_change") {
    curve_ = boost::none;
    current_s_ = 0;
    return BT::NodeStatus::FAILURE;
  }
  if (!to_lanelet_id_) {
    curve_ = boost::none;
    current_s_ = 0;
    return BT::NodeStatus::FAILURE;
  }
  if (!curve_) {
    if (request == "lane_change") {
      if (!hdmap_utils->canChangeLane(
          entity_status.lanelet_pose.lanelet_id,
          to_lanelet_id_.get()))
      {
        return BT::NodeStatus::FAILURE;
      }
      auto from_pose = hdmap_utils->toMapPose(entity_status.lanelet_pose).pose;
      auto ret = hdmap_utils->getLaneChangeTrajectory(from_pose, to_lanelet_id_.get());
      if (ret) {
        curve_ = ret->first;
        target_s_ = ret->second;
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }
  }
  if (curve_) {
    double current_linear_vel = entity_status.action_status.twist.linear.x;
    current_s_ = current_s_ + current_linear_vel * step_time;
    if (current_s_ < curve_->getLength()) {
      geometry_msgs::msg::Pose pose = curve_->getPose(current_s_, true);
      openscenario_msgs::msg::EntityStatus entity_status_updated;
      entity_status_updated.pose = pose;
      auto lanelet_pose = hdmap_utils->toLaneletPose(pose);
      if (lanelet_pose) {
        entity_status_updated.lanelet_pose = lanelet_pose.get();
      } else {
        entity_status_updated.lanelet_pose_valid = false;
      }
      entity_status_updated.action_status = entity_status.action_status;
      setOutput("updated_status", entity_status_updated);
      const auto waypoints = calculateWaypoints();
      const auto obstacle = calculateObstacle(waypoints);
      setOutput("waypoints", waypoints);
      setOutput("obstacle", obstacle);
      return BT::NodeStatus::RUNNING;
    } else {
      const auto waypoints = calculateWaypoints();
      const auto obstacle = calculateObstacle(waypoints);
      setOutput("waypoints", waypoints);
      setOutput("obstacle", obstacle);
      double s = (current_s_ - curve_->getLength()) + target_s_;
      curve_ = boost::none;
      current_s_ = 0;
      openscenario_msgs::msg::EntityStatus entity_status_updated;
      openscenario_msgs::msg::LaneletPose lanelet_pose;
      lanelet_pose.lanelet_id = to_lanelet_id_.get();
      lanelet_pose.s = s;
      lanelet_pose.offset = 0;
      entity_status_updated.pose = hdmap_utils->toMapPose(lanelet_pose).pose;
      entity_status_updated.lanelet_pose = lanelet_pose;
      entity_status_updated.action_status = entity_status.action_status;
      setOutput("updated_status", entity_status_updated);
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}
}      // namespace vehicle
}  // namespace entity_behavior
