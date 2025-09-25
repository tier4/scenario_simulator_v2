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
  auto parameterToSeeAroundMode = [](std::string_view parameter) {
    if (parameter == "blind") {
      return SeeAroundMode::blind;
    } else if (parameter == "aware") {
      return SeeAroundMode::aware;
    } else {
      THROW_SIMULATION_ERROR("Unknown see_around mode. It must be \"blind\" or \"aware\".");
    }
  };

  should_respect_see_around = parameterToSeeAroundMode(
    common::getParameter<std::string>("pedestrian_ignore_see_around", "blind"));
}

void WalkStraightAction::getBlackBoardValues() { PedestrianActionNode::getBlackBoardValues(); }

bool checkPointIsInfront(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Point & point)
{
  const auto self_yaw = math::geometry::convertQuaternionToEulerAngle(pose.orientation).z;
  const auto dx = point.x - pose.position.x;
  const auto dy = point.y - pose.position.y;
  const auto vec_yaw = std::atan2(dy, dx);
  const auto yaw_diff = std::atan2(std::sin(vec_yaw - self_yaw), std::cos(vec_yaw - self_yaw));

  return std::fabs(yaw_diff) <= boost::math::constants::half_pi<double>();
}

bool WalkStraightAction::detectObstacleInFront(const bool see_around) const
{
  if (should_respect_see_around == SeeAroundMode::blind) {
    return false;
  }

  if (!see_around) {
    return false;
  }

  auto hasObstacleInFrontOfPedestrian = [this](double distance) {
    using math::geometry::operator-;
    using math::geometry::operator*;

    const auto & pedestrian_pose = canonicalized_entity_status_->getMapPose();

    for (const auto & [_, entity_status] : other_entity_status_) {
      const auto & other_position = entity_status.getMapPose().position;
      const auto norm = math::geometry::norm(other_position - pedestrian_pose.position);
      if (norm > distance) continue;

      if (checkPointIsInfront(pedestrian_pose, other_position)) {
        // is in front of pedestrian check if collide
        const auto bounding_box_map_points = math::geometry::transformPoints(
          pedestrian_pose,
          math::geometry::getPointsFromBbox(canonicalized_entity_status_->getBoundingBox()));

        std::vector<geometry_msgs::msg::Point> front_points;
        std::vector<geometry_msgs::msg::Point> check_area_points;
        for (const auto & point : bounding_box_map_points) {
          if (checkPointIsInfront(pedestrian_pose, point)) {
            front_points.push_back(point);
          }
        }
        auto orientation_vector =
          math::geometry::normalize(
            math::geometry::convertQuaternionToEulerAngle(pedestrian_pose.orientation)) *
          distance;
        for (auto & point : front_points) {
          check_area_points.push_back(point);
          point.x += orientation_vector.x;
          point.y += orientation_vector.z;
          point.z += orientation_vector.y;
          check_area_points.push_back(point);
        }
        const auto check_polygon = math::geometry::toBoostPolygon(check_area_points);
        const auto poly =
          math::geometry::toPolygon2D(entity_status.getMapPose(), entity_status.getBoundingBox());
        if (
          boost::geometry::intersects(check_polygon, poly) ||
          boost::geometry::intersects(poly, check_polygon)) {
          return true;
        } else if (boost::geometry::disjoint(check_polygon, poly)) {
          return false;
        }
        return true;
      }
    }
    return false;
  };

  const double min_stop = calculateStopDistance(behavior_parameter_.dynamic_constraints);
  const double stable_distance =
    min_stop + canonicalized_entity_status_->getBoundingBox().dimensions.x;

  return hasObstacleInFrontOfPedestrian(stable_distance);
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

  const auto obstacle_detector_result = detectObstacleInFront(behavior_parameter_.see_around);
  target_speed_ = obstacle_detector_result ? 0.0 : target_speed_;

  setCanonicalizedEntityStatus(calculateUpdatedEntityStatusInWorldFrame(target_speed_.value()));
  return BT::NodeStatus::RUNNING;
}
}  // namespace pedestrian
}  // namespace entity_behavior
