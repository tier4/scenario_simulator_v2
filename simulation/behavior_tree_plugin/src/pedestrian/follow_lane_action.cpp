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
#include <behavior_tree_plugin/pedestrian/follow_lane_action.hpp>
#include <cmath>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <traffic_simulator_msgs/msg/obstacle.hpp>
#include <vector>

namespace entity_behavior
{
namespace pedestrian
{
FollowLaneAction::FollowLaneAction(const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::PedestrianActionNode(name, config)
{
  use_trajectory_based_front_entity_detection_ =
    common::getParameter<bool>("use_trajectory_based_front_entity_detection", false);
  trajectory_based_detection_offset_ =
    common::getParameter<double>("trajectory_based_detection_offset", 0.0);
}

void FollowLaneAction::getBlackBoardValues() { PedestrianActionNode::getBlackBoardValues(); }

bool FollowLaneAction::detectObstacleInLane(
  const lanelet::Ids pedestrian_lanes, const bool see_around,
  const std::vector<geometry_msgs::msg::Point> & waypoints) const
{
  if (should_respect_see_around == SeeAroundMode::blind) {
    return false;
  }

  if (!see_around) {
    return false;
  }

  auto hasObstacleInPedestrianLanes =
    [this](const lanelet::Ids pedestrian_lanes_local, const double max_detect_length) {
      using math::geometry::operator-;
      const auto & pedestrian_position = canonicalized_entity_status_->getMapPose().position;
      lanelet::Ids other_entity_lane_ids;
      for (const auto & [_, status] : other_entity_status_) {
        if (status.getType().type != traffic_simulator_msgs::msg::EntityType::EGO) {
          continue;
        }
        if (!status.isInLanelet()) {
          continue;
        }
        const auto norm = math::geometry::norm(status.getMapPose().position - pedestrian_position);
        if (!(norm < max_detect_length)) {
          continue;
        }
        other_entity_lane_ids.push_back(status.getLaneletId());
      }
      std::unordered_set<lanelet::Id> other_lane_id_set(
        other_entity_lane_ids.begin(), other_entity_lane_ids.end());
      for (const auto & pedestrian_lane : pedestrian_lanes_local) {
        if (other_lane_id_set.count(pedestrian_lane)) {
          return true;
        }
      }
      return false;
    };

  auto hasObstacleInFrontOfPedestrian = [this]() {
    using math::geometry::operator-;

    const auto & pedestrian_position = canonicalized_entity_status_->getMapPose().position;

    for (const auto & [_, entity_status] : other_entity_status_) {
      const auto & other_position = entity_status.getMapPose().position;
      const auto relative_position = other_position - pedestrian_position;
      const double relative_angle_rad = std::atan2(relative_position.y, relative_position.x);
      if (relative_angle_rad > 0) {
        return true;
      }
    }
    return false;
  };

  if (use_trajectory_based_front_entity_detection_) {
    constexpr std::size_t trajectory_segments = 50;
    if (
      const auto front_entity_info = getFrontEntityNameAndDistanceByTrajectory(
        waypoints,
        pedestrian_parameters.bounding_box.dimensions.y + trajectory_based_detection_offset_,
        trajectory_segments)) {
      return true;
    } else {
      return false;
    }
  } else {
    if (hasObstacleInPedestrianLanes(pedestrian_lanes, 10) && hasObstacleInFrontOfPedestrian()) {
      return true;
    } else {
      return false;
    }
  }
}

traffic_simulator_msgs::msg::WaypointsArray FollowLaneAction::calculateWaypoints(
  const lanelet::Ids & following_lanelets) const
{
  traffic_simulator_msgs::msg::WaypointsArray waypoints;
  if (!canonicalized_entity_status_->isInLanelet()) {
    return waypoints;
  }
  if (following_lanelets.empty()) {
    waypoints.waypoints.push_back(canonicalized_entity_status_->getMapPose().position);
    return waypoints;
  }

  const auto center_points = hdmap_utils_->getCenterPoints(following_lanelets);
  if (center_points.size() < 2) {
    waypoints.waypoints.push_back(canonicalized_entity_status_->getMapPose().position);
    return waypoints;
  }

  math::geometry::CatmullRomSpline spline(center_points);
  const auto lanelet_pose = canonicalized_entity_status_->getLaneletPose();
  const double start_s = std::clamp(lanelet_pose.s, 0.0, spline.getLength());
  const double end_s = std::min(start_s + getHorizon(), spline.getLength());
  constexpr double interval = 1.0;

  auto append_if_different = [&waypoints](const geometry_msgs::msg::Point & point) -> void {
    if (waypoints.waypoints.empty()) {
      waypoints.waypoints.push_back(point);
      return;
    }
    const auto & last = waypoints.waypoints.back();
    constexpr double epsilon = 1.0e-3;
    if (
      std::hypot(last.x - point.x, last.y - point.y) > epsilon ||
      std::fabs(last.z - point.z) > epsilon) {
      waypoints.waypoints.push_back(point);
    }
  };

  append_if_different(canonicalized_entity_status_->getMapPose().position);
  for (double s = start_s; s < end_s; s += interval) {
    append_if_different(spline.getPoint(s));
  }
  append_if_different(spline.getPoint(end_s));

  return waypoints;
}

bool FollowLaneAction::checkPreconditions()
{
  if (
    request_ != traffic_simulator::behavior::Request::NONE &&
    request_ != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return false;
  } else {
    return true;
  }
}

BT::NodeStatus FollowLaneAction::doAction()
{
  if (!canonicalized_entity_status_->isInLanelet()) {
    stopEntity();
    setOutput("waypoints", traffic_simulator_msgs::msg::WaypointsArray());
    setOutput("obstacle", std::optional<traffic_simulator_msgs::msg::Obstacle>());
    return BT::NodeStatus::RUNNING;
  }
  auto following_lanelets = hdmap_utils_->getFollowingLanelets(
    canonicalized_entity_status_->getLaneletId(), getHorizon(), true);
  if (!target_speed_) {
    target_speed_ = hdmap_utils_->getSpeedLimit(following_lanelets);
  }
  const auto waypoints = calculateWaypoints(following_lanelets);
  const auto obstacle_detector_result =
    detectObstacleInLane(following_lanelets, behavior_parameter_.see_around, waypoints.waypoints);
  target_speed_ = obstacle_detector_result ? 0.0 : target_speed_;

  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed_.value()));
  setOutput("waypoints", waypoints);
  setOutput("obstacle", std::optional<traffic_simulator_msgs::msg::Obstacle>());
  return BT::NodeStatus::RUNNING;
}
}  // namespace pedestrian
}  // namespace entity_behavior
