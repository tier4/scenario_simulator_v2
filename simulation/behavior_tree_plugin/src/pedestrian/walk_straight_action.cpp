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

#include <behavior_tree_plugin/pedestrian/walk_straight_action.hpp>
#include <geometry/bounding_box.hpp>
#include <geometry/distance.hpp>
#include <geometry/quaternion/get_normal_vector.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/transform.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <string>

namespace entity_behavior
{
namespace pedestrian
{
WalkStraightAction::WalkStraightAction(
  const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::PedestrianActionNode(name, config)
{
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
  using math::geometry::operator*;

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

  const auto detection_horizon_vector =
    math::geometry::normalize(
      math::geometry::convertQuaternionToEulerAngle(pedestrian_pose.orientation)) *
    detection_horizon;
  std::vector<geometry_msgs::msg::Point> detection_area_points;

  for (const auto & point : bounding_box_front_points) {
    detection_area_points.push_back(point);
    geometry_msgs::msg::Point front_detection_point;
    front_detection_point.x = point.x + detection_horizon_vector.x;
    front_detection_point.y = point.y + detection_horizon_vector.z;
    front_detection_point.z = point.z + detection_horizon_vector.y;
    detection_area_points.push_back(front_detection_point);
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

bool WalkStraightAction::detectObstacleInFront(const bool see_around) const
{
  auto isObstacleInFrontOfPedestrian = [this](const double & detection_horizon) {
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

  if (not see_around || should_respect_see_around == SeeAroundMode::blind) {
    return false;
  } else {
    const double detection_horizon =
      calculateStopDistance(behavior_parameter_.dynamic_constraints) +
      canonicalized_entity_status_->getBoundingBox().dimensions.x;
    return isObstacleInFrontOfPedestrian(detection_horizon);
  }
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

  const auto is_obstacle_in_front = detectObstacleInFront(behavior_parameter_.see_around);
  target_speed_ = is_obstacle_in_front ? 0.0 : target_speed_;

  setCanonicalizedEntityStatus(calculateUpdatedEntityStatusInWorldFrame(target_speed_.value()));
  return BT::NodeStatus::RUNNING;
}
}  // namespace pedestrian
}  // namespace entity_behavior
