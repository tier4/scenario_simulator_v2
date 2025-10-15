/**
 * @file walk_straight_action.cpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief class implementation of the walk straight action
 * @version 0.1
 * @date 2021-04-02
 *
 * @copyright Copyright(c) TIER IV.Inc {2015}
 *
 */

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
#include <behavior_tree_plugin/pedestrian/walk_straight_action.hpp>
#include <cmath>
#include <geometry/bounding_box.hpp>
#include <geometry/distance.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/transform.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/operator.hpp>
#include <get_parameter/get_parameter.hpp>
#include <limits>
#include <optional>
#include <string>
#include <traffic_simulator_msgs/msg/obstacle.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>

namespace entity_behavior
{
namespace pedestrian
{
WalkStraightAction::WalkStraightAction(
  const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::PedestrianActionNode(name, config)
{
  use_trajectory_based_front_entity_detection_ =
    common::getParameter<bool>("use_trajectory_based_front_entity_detection", false);
}

void WalkStraightAction::getBlackBoardValues() { PedestrianActionNode::getBlackBoardValues(); }

bool isPointInFront(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Point & target_point)
{
  const auto self_yaw = math::geometry::convertQuaternionToEulerAngle(pose.orientation).z;
  const auto angle_to_target_point =
    std::atan2(target_point.y - pose.position.y, target_point.x - pose.position.x);
  const auto yaw_difference = std::atan2(
    std::sin(angle_to_target_point - self_yaw), std::cos(angle_to_target_point - self_yaw));
  return std::fabs(yaw_difference) <= boost::math::constants::half_pi<double>();
}

bool WalkStraightAction::isEntityColliding(
  const traffic_simulator::entity_status::CanonicalizedEntityStatus & entity_status,
  const double & detection_horizon) const
{
  auto translatePoint =
    [](const geometry_msgs::msg::Point & p, const double yaw, const double distance) {
      geometry_msgs::msg::Point result;

      result.x = p.x + distance * std::cos(yaw);
      result.y = p.y + distance * std::sin(yaw);
      result.z = p.z;

      return result;
    };

  const auto & pedestrian_pose = canonicalized_entity_status_->getMapPose();
  const auto bounding_box_map_points = math::geometry::transformPoints(
    pedestrian_pose,
    math::geometry::getPointsFromBbox(canonicalized_entity_status_->getBoundingBox()));
  std::vector<geometry_msgs::msg::Point> bounding_box_front_points;
  for (const auto & point : bounding_box_map_points) {
    if (isPointInFront(pedestrian_pose, point)) {
      bounding_box_front_points.push_back(point);
    }
  }
  const auto self_yaw =
    math::geometry::convertQuaternionToEulerAngle(pedestrian_pose.orientation).z;
  std::vector<geometry_msgs::msg::Point> detection_area_points;

  // For the two front bounding box points, first add the original point, then a point
  // shifted forward in the pedestrian's facing direction (self_yaw and detection_horizon).
  // The original points define the base of the detection area, while the shifted points
  // define the far edge, together forming the detection zone in front of the pedestrian
  for (const auto & point : bounding_box_front_points) {
    detection_area_points.push_back(point);
    detection_area_points.push_back(translatePoint(point, self_yaw, detection_horizon));
  }

  const auto detection_area_polygon = math::geometry::toBoostPolygon(detection_area_points);
  const auto other_entity_polygon =
    math::geometry::toPolygon2D(entity_status.getMapPose(), entity_status.getBoundingBox());

  if (
    boost::geometry::intersects(detection_area_polygon, other_entity_polygon) ||
    boost::geometry::intersects(other_entity_polygon, detection_area_polygon)) {
    return true;
  } else if (boost::geometry::disjoint(detection_area_polygon, other_entity_polygon)) {
    return false;
  } else {
    return true;
  }
}

bool WalkStraightAction::isObstacleInFront(
  const bool see_around, const std::vector<geometry_msgs::msg::Point> waypoints) const
{
  if (not see_around || should_respect_see_around == SeeAroundMode::blind) {
    return false;
  }

  if (use_trajectory_based_front_entity_detection_) {
    constexpr std::size_t trajectory_segments = 50;
    if (
      const auto front_entity_info = getFrontEntityNameAndDistanceByTrajectory(
        waypoints, canonicalized_entity_status_->getBoundingBox().dimensions.y,
        trajectory_segments)) {
      return true;
    } else {
      return false;
    }
  } else {
    auto isObstacleInFrontOfPedestrian = [this](const double detection_horizon) {
      using math::geometry::operator-;
      const auto & pedestrian_pose = canonicalized_entity_status_->getMapPose();
      for (const auto & [_, entity_status] : other_entity_status_) {
        const auto & other_position = entity_status.getMapPose().position;
        if (const auto distance = math::geometry::norm(other_position - pedestrian_pose.position);
            distance <= detection_horizon) {
          if (
            isPointInFront(pedestrian_pose, other_position) &&
            isEntityColliding(entity_status, detection_horizon)) {
            return true;
          }
        }
      }
      return false;
    };

    const double detection_horizon =
      calculateStopDistance(behavior_parameter_.dynamic_constraints) +
      canonicalized_entity_status_->getBoundingBox().dimensions.x + front_entity_margin;
    return isObstacleInFrontOfPedestrian(detection_horizon);
  }
}

auto WalkStraightAction::calculateWaypoints() const -> traffic_simulator_msgs::msg::WaypointsArray
{
  traffic_simulator_msgs::msg::WaypointsArray waypoints;
  const auto pose = canonicalized_entity_status_->getMapPose();

  auto append_if_different = [&waypoints](const geometry_msgs::msg::Point & point) {
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

  append_if_different(pose.position);

  if (canonicalized_entity_status_->isInLanelet()) {
    auto lanelets_to_follow = route_lanelets_;
    if (lanelets_to_follow.empty()) {
      lanelets_to_follow = hdmap_utils_->getFollowingLanelets(
        canonicalized_entity_status_->getLaneletId(), getHorizon(), true);
    }

    if (!lanelets_to_follow.empty()) {
      const auto center_points = hdmap_utils_->getCenterPoints(lanelets_to_follow);
      if (center_points.size() >= 2) {
        math::geometry::CatmullRomSpline spline(center_points);
        const auto lanelet_pose = canonicalized_entity_status_->getLaneletPose();
        const double start_s = std::clamp(lanelet_pose.s, 0.0, spline.getLength());
        const double end_s = std::min(start_s + getHorizon(), spline.getLength());
        constexpr double interval = 1.0;

        for (double s = start_s; s < end_s; s += interval) {
          append_if_different(spline.getPoint(s));
        }
        append_if_different(spline.getPoint(end_s));
        return waypoints;
      }
    }
  }

  const auto yaw = math::geometry::convertQuaternionToEulerAngle(pose.orientation).z;
  const double horizon = getHorizon();
  constexpr double interval = 1.0;

  geometry_msgs::msg::Point point = pose.position;
  double accumulated = 0.0;
  while (accumulated + interval <= horizon) {
    point.x += interval * std::cos(yaw);
    point.y += interval * std::sin(yaw);
    append_if_different(point);
    accumulated += interval;
  }

  const double remaining = horizon - accumulated;
  if (remaining > std::numeric_limits<double>::epsilon()) {
    point.x += remaining * std::cos(yaw);
    point.y += remaining * std::sin(yaw);
    append_if_different(point);
  }

  return waypoints;
}

bool WalkStraightAction::checkPreconditions()
{
  return request_ == traffic_simulator::behavior::Request::WALK_STRAIGHT;
}

BT::NodeStatus WalkStraightAction::doAction()
{
  if (!target_speed_) {
    target_speed_ = 1.111;
  }

  const auto waypoints = calculateWaypoints();
  target_speed_ =
    isObstacleInFront(behavior_parameter_.see_around, waypoints.waypoints) ? 0.0 : target_speed_;
  setCanonicalizedEntityStatus(calculateUpdatedEntityStatusInWorldFrame(target_speed_.value()));
  setOutput("waypoints", waypoints);
  setOutput("obstacle", std::optional<traffic_simulator_msgs::msg::Obstacle>());
  return BT::NodeStatus::RUNNING;
}
}  // namespace pedestrian
}  // namespace entity_behavior
