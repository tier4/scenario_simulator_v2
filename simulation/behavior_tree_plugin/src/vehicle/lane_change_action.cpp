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
#include <behavior_tree_plugin/vehicle/behavior_tree.hpp>
#include <behavior_tree_plugin/vehicle/lane_change_action.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/transform.hpp>
#include <memory>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <vector>

namespace entity_behavior
{
namespace vehicle
{
LaneChangeAction::LaneChangeAction(const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config), current_s_(0)
{
}

const std::optional<traffic_simulator_msgs::msg::Obstacle> LaneChangeAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
{
  return std::nullopt;
}

const traffic_simulator_msgs::msg::WaypointsArray LaneChangeAction::calculateWaypoints()
{
  if (!curve_) {
    THROW_SIMULATION_ERROR("curve is null");
  }
  if (!lane_change_parameters_) {
    THROW_SIMULATION_ERROR("lane change parameter is null");
  }
  if (canonicalized_entity_status->getTwist().linear.x >= 0) {
    traffic_simulator_msgs::msg::WaypointsArray waypoints;
    double horizon = getHorizon();
    auto following_lanelets =
      hdmap_utils->getFollowingLanelets(lane_change_parameters_->target.lanelet_id, 0);
    double l = curve_->getLength();
    double rest_s = current_s_ + horizon - l;
    if (rest_s < 0) {
      const auto curve_waypoints =
        curve_->getTrajectory(current_s_, current_s_ + horizon, 1.0, true);
      waypoints.waypoints = curve_waypoints;
    } else {
      std::vector<geometry_msgs::msg::Point> center_points =
        hdmap_utils->getCenterPoints(following_lanelets);
      // DIFFERENT SPLINE - recalculation needed
      math::geometry::CatmullRomSpline spline(center_points);
      const auto straight_waypoints = spline.getTrajectory(target_s_, target_s_ + rest_s, 1.0);
      waypoints.waypoints = straight_waypoints;
      const auto curve_waypoints = curve_->getTrajectory(current_s_, l, 1.0, true);
      std::copy(
        straight_waypoints.begin(), straight_waypoints.end(),
        std::back_inserter(waypoints.waypoints));
    }
    return waypoints;
  } else {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
}

void LaneChangeAction::getBlackBoardValues()
{
  VehicleActionNode::getBlackBoardValues();
  traffic_simulator::lane_change::Parameter lane_change_parameters;
  if (!getInput<traffic_simulator::lane_change::Parameter>(
        "lane_change_parameters", lane_change_parameters)) {
    lane_change_parameters_ = std::nullopt;
  } else {
    lane_change_parameters_ = lane_change_parameters;
  }
}

BT::NodeStatus LaneChangeAction::tick()
{
  getBlackBoardValues();
  if (request != traffic_simulator::behavior::Request::LANE_CHANGE) {
    curve_ = std::nullopt;
    current_s_ = 0;
    return BT::NodeStatus::FAILURE;
  }
  if (!lane_change_parameters_) {
    curve_ = std::nullopt;
    current_s_ = 0;
    return BT::NodeStatus::FAILURE;
  }
  if (!curve_) {
    if (request == traffic_simulator::behavior::Request::LANE_CHANGE) {
      if (!canonicalized_entity_status->laneMatchingSucceed()) {
        return BT::NodeStatus::FAILURE;
      }
      const auto lanelet_pose = canonicalized_entity_status->getLaneletPose();
      if (!hdmap_utils->canChangeLane(
            lanelet_pose.lanelet_id, lane_change_parameters_->target.lanelet_id)) {
        return BT::NodeStatus::FAILURE;
      }
      std::optional<std::pair<math::geometry::HermiteCurve, double>> traj_with_goal;
      traffic_simulator::LaneletPose along_pose, goal_pose;
      switch (lane_change_parameters_->constraint.type) {
        case traffic_simulator::lane_change::Constraint::Type::NONE:
          /**
          @note Hard coded parameter,
          10.0 is a maximum_curvature_threshold (If the curvature of the trajectory is over 10.0, the trajectory was not selected.)
          20.0 is a target_trajectory_length (The one with the closest length to 20 m is selected from the candidate trajectories.)
          1.0 is a forward_distance_threshold (If the goal x position in the cartesian coordinate was under 1.0, the goal was rejected.)
          */
          traj_with_goal = hdmap_utils->getLaneChangeTrajectory(
            hdmap_utils->toMapPose(lanelet_pose).pose, lane_change_parameters_.value(), 10.0, 20.0,
            1.0);
          along_pose = hdmap_utils->getAlongLaneletPose(
            lanelet_pose, traffic_simulator::lane_change::Parameter::default_lanechange_distance);
          break;
        case traffic_simulator::lane_change::Constraint::Type::LATERAL_VELOCITY:
          traj_with_goal =
            hdmap_utils->getLaneChangeTrajectory(lanelet_pose, lane_change_parameters_.value());
          along_pose = hdmap_utils->getAlongLaneletPose(
            lanelet_pose, traffic_simulator::lane_change::Parameter::default_lanechange_distance);
          break;
        case traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE:
          traj_with_goal =
            hdmap_utils->getLaneChangeTrajectory(lanelet_pose, lane_change_parameters_.value());
          along_pose = hdmap_utils->getAlongLaneletPose(
            lanelet_pose, lane_change_parameters_->constraint.value);
          break;
        case traffic_simulator::lane_change::Constraint::Type::TIME:
          traj_with_goal =
            hdmap_utils->getLaneChangeTrajectory(lanelet_pose, lane_change_parameters_.value());
          along_pose = hdmap_utils->getAlongLaneletPose(
            lanelet_pose, lane_change_parameters_->constraint.value);
          break;
      }
      if (traj_with_goal) {
        curve_ = traj_with_goal->first;
        target_s_ = traj_with_goal->second;
        goal_pose.lanelet_id = lane_change_parameters_->target.lanelet_id;
        goal_pose.s = traj_with_goal->second;
        double offset = std::fabs(
          math::geometry::getRelativePose(
            hdmap_utils->toMapPose(along_pose).pose, hdmap_utils->toMapPose(goal_pose).pose)
            .position.y);
        switch (lane_change_parameters_->constraint.type) {
          case traffic_simulator::lane_change::Constraint::Type::NONE:
            lane_change_velocity_ = canonicalized_entity_status->getTwist().linear.x;
            break;
          case traffic_simulator::lane_change::Constraint::Type::LATERAL_VELOCITY:
            lane_change_velocity_ =
              curve_->getLength() / (offset / lane_change_parameters_->constraint.value);
            break;
          case traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE:
            lane_change_velocity_ = canonicalized_entity_status->getTwist().linear.x;
            break;
          case traffic_simulator::lane_change::Constraint::Type::TIME:
            lane_change_velocity_ = curve_->getLength() / lane_change_parameters_->constraint.value;
            break;
        }
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }
  }
  if (curve_) {
    double target_accel = 0;
    switch (lane_change_parameters_->constraint.policy) {
      /**
       * @brief Force changing speed in order to fulfill constraint.
       */
      case traffic_simulator::lane_change::Constraint::Policy::FORCE:
        canonicalized_entity_status->setTwist(geometry_msgs::msg::Twist());
        canonicalized_entity_status->setAccel(geometry_msgs::msg::Accel());
        canonicalized_entity_status->setLinearVelocity(lane_change_velocity_);
        current_s_ = current_s_ + canonicalized_entity_status->getTwist().linear.x * step_time;
        break;
      /**
       * @brief Changing linear speed and try to fulfill constraint.
       */
      case traffic_simulator::lane_change::Constraint::Policy::BEST_EFFORT:
        target_accel =
          (lane_change_velocity_ - canonicalized_entity_status->getTwist().linear.x) / step_time;
        if (canonicalized_entity_status->getTwist().linear.x > target_speed) {
          target_accel = std::clamp(
            target_accel, behavior_parameter.dynamic_constraints.max_deceleration * -1.0, 0.0);
        } else {
          target_accel =
            std::clamp(target_accel, 0.0, behavior_parameter.dynamic_constraints.max_acceleration);
        }
        geometry_msgs::msg::Accel accel_new;
        accel_new.linear.x = target_accel;
        geometry_msgs::msg::Twist twist_new;
        /**
         * @note Hard coded parameter, -10.0 is a minimum linear velocity of the entity.
        */
        twist_new.linear.x = std::clamp(
          canonicalized_entity_status->getTwist().linear.x + accel_new.linear.x * step_time, -10.0,
          vehicle_parameters.performance.max_speed);
        twist_new.linear.y = 0.0;
        twist_new.linear.z = 0.0;
        twist_new.angular.x = 0.0;
        twist_new.angular.y = 0.0;
        twist_new.angular.z = 0.0;
        canonicalized_entity_status->setTwist(twist_new);
        canonicalized_entity_status->setAccel(accel_new);
        current_s_ = current_s_ + canonicalized_entity_status->getTwist().linear.x * step_time;
        break;
    }
    if (current_s_ < curve_->getLength()) {
      geometry_msgs::msg::Pose pose = curve_->getPose(current_s_, true);
      auto entity_status_updated =
        static_cast<traffic_simulator::EntityStatus>(*canonicalized_entity_status);
      entity_status_updated.pose = pose;
      entity_status_updated.lanelet_pose_valid = false;
      entity_status_updated.action_status = canonicalized_entity_status->getActionStatus();
      setCanonicalizedEntityStatus(entity_status_updated);
      const auto waypoints = calculateWaypoints();
      if (waypoints.waypoints.empty()) {
        return BT::NodeStatus::FAILURE;
      }
      const auto obstacle = calculateObstacle(waypoints);
      setOutput("waypoints", waypoints);
      setOutput("obstacle", obstacle);
      return BT::NodeStatus::RUNNING;
    } else {
      const auto waypoints = calculateWaypoints();
      if (waypoints.waypoints.empty()) {
        return BT::NodeStatus::FAILURE;
      }
      const auto obstacle = calculateObstacle(waypoints);
      setOutput("waypoints", waypoints);
      setOutput("obstacle", obstacle);
      double s = (current_s_ - curve_->getLength()) + target_s_;
      curve_ = std::nullopt;
      current_s_ = 0;
      lane_change_velocity_ = 0;
      auto entity_status_updated =
        static_cast<traffic_simulator::EntityStatus>(*canonicalized_entity_status);
      traffic_simulator::LaneletPose lanelet_pose;
      lanelet_pose.lanelet_id = lane_change_parameters_->target.lanelet_id;
      lanelet_pose.s = s;
      lanelet_pose.offset = 0;
      entity_status_updated.lanelet_pose = lanelet_pose;
      entity_status_updated.lanelet_pose_valid = true;
      entity_status_updated.pose = hdmap_utils->toMapPose(lanelet_pose).pose;
      entity_status_updated.action_status = canonicalized_entity_status->getActionStatus();
      setCanonicalizedEntityStatus(entity_status_updated);
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace vehicle
}  // namespace entity_behavior
