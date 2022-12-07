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

const boost::optional<traffic_simulator_msgs::msg::Obstacle> LaneChangeAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
{
  return boost::none;
}

const traffic_simulator_msgs::msg::WaypointsArray LaneChangeAction::calculateWaypoints()
{
  if (!curve_) {
    THROW_SIMULATION_ERROR("curve is null");
  }
  if (!lane_change_parameters_) {
    THROW_SIMULATION_ERROR("lane change parameter is null");
  }
  if (entity_status.action_status.twist.linear.x >= 0) {
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
    lane_change_parameters_ = boost::none;
  } else {
    lane_change_parameters_ = lane_change_parameters;
  }
}

BT::NodeStatus LaneChangeAction::tick()
{
  getBlackBoardValues();
  if (request != traffic_simulator::behavior::Request::LANE_CHANGE) {
    curve_ = boost::none;
    current_s_ = 0;
    return BT::NodeStatus::FAILURE;
  }
  if (!lane_change_parameters_) {
    curve_ = boost::none;
    current_s_ = 0;
    return BT::NodeStatus::FAILURE;
  }
  if (!curve_) {
    if (request == traffic_simulator::behavior::Request::LANE_CHANGE) {
      if (!hdmap_utils->canChangeLane(
            entity_status.lanelet_pose.lanelet_id, lane_change_parameters_->target.lanelet_id)) {
        return BT::NodeStatus::FAILURE;
      }
      boost::optional<std::pair<math::geometry::HermiteCurve, double>> traj_with_goal;
      traffic_simulator_msgs::msg::LaneletPose along_pose, goal_pose;
      switch (lane_change_parameters_->constraint.type) {
        case traffic_simulator::lane_change::Constraint::Type::NONE:
          traj_with_goal = hdmap_utils->getLaneChangeTrajectory(
            hdmap_utils->toMapPose(entity_status.lanelet_pose).pose, lane_change_parameters_.get(),
            10.0, 20.0, 1.0);
          along_pose = hdmap_utils->getAlongLaneletPose(
            entity_status.lanelet_pose,
            traffic_simulator::lane_change::Parameter::default_lanechange_distance);
          break;
        case traffic_simulator::lane_change::Constraint::Type::LATERAL_VELOCITY:
          traj_with_goal = hdmap_utils->getLaneChangeTrajectory(
            entity_status.lanelet_pose, lane_change_parameters_.get());
          along_pose = hdmap_utils->getAlongLaneletPose(
            entity_status.lanelet_pose,
            traffic_simulator::lane_change::Parameter::default_lanechange_distance);
          break;
        case traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE:
          traj_with_goal = hdmap_utils->getLaneChangeTrajectory(
            entity_status.lanelet_pose, lane_change_parameters_.get());
          along_pose = hdmap_utils->getAlongLaneletPose(
            entity_status.lanelet_pose, lane_change_parameters_->constraint.value);
          break;
        case traffic_simulator::lane_change::Constraint::Type::TIME:
          traj_with_goal = hdmap_utils->getLaneChangeTrajectory(
            entity_status.lanelet_pose, lane_change_parameters_.get());
          along_pose = hdmap_utils->getAlongLaneletPose(
            entity_status.lanelet_pose, lane_change_parameters_->constraint.value);
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
            lane_change_velocity_ = entity_status.action_status.twist.linear.x;
            break;
          case traffic_simulator::lane_change::Constraint::Type::LATERAL_VELOCITY:
            lane_change_velocity_ =
              curve_->getLength() / (offset / lane_change_parameters_->constraint.value);
            break;
          case traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE:
            lane_change_velocity_ = entity_status.action_status.twist.linear.x;
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
        entity_status.action_status.twist = geometry_msgs::msg::Twist();
        entity_status.action_status.accel = geometry_msgs::msg::Accel();
        entity_status.action_status.twist.linear.x = lane_change_velocity_;
        current_s_ = current_s_ + entity_status.action_status.twist.linear.x * step_time;
        break;
      /**
       * @brief Changing linear speed and try to fulfill constraint.
       */
      case traffic_simulator::lane_change::Constraint::Policy::BEST_EFFORT:
        target_accel =
          (lane_change_velocity_ - entity_status.action_status.twist.linear.x) / step_time;
        if (entity_status.action_status.twist.linear.x > target_speed) {
          target_accel = std::clamp(
            target_accel, behavior_parameter.dynamic_constraints.max_deceleration * -1.0, 0.0);
        } else {
          target_accel =
            std::clamp(target_accel, 0.0, behavior_parameter.dynamic_constraints.max_acceleration);
        }
        geometry_msgs::msg::Accel accel_new;
        accel_new.linear.x = target_accel;
        geometry_msgs::msg::Twist twist_new;
        twist_new.linear.x = std::clamp(
          entity_status.action_status.twist.linear.x + accel_new.linear.x * step_time, -10.0,
          vehicle_parameters.performance.max_speed);
        twist_new.linear.y = 0.0;
        twist_new.linear.z = 0.0;
        twist_new.angular.x = 0.0;
        twist_new.angular.y = 0.0;
        twist_new.angular.z = 0.0;
        entity_status.action_status.twist = twist_new;
        entity_status.action_status.accel = accel_new;
        current_s_ = current_s_ + entity_status.action_status.twist.linear.x * step_time;
        break;
    }
    if (current_s_ < curve_->getLength()) {
      geometry_msgs::msg::Pose pose = curve_->getPose(current_s_, true);
      traffic_simulator_msgs::msg::EntityStatus entity_status_updated;
      entity_status_updated.pose = pose;
      auto lanelet_pose = hdmap_utils->toLaneletPose(pose, entity_status.bounding_box, false);
      if (lanelet_pose) {
        entity_status_updated.lanelet_pose = lanelet_pose.get();
      } else {
        entity_status_updated.lanelet_pose_valid = false;
      }
      entity_status_updated.action_status = entity_status.action_status;
      setOutput("updated_status", entity_status_updated);
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
      curve_ = boost::none;
      current_s_ = 0;
      lane_change_velocity_ = 0;
      traffic_simulator_msgs::msg::EntityStatus entity_status_updated;
      traffic_simulator_msgs::msg::LaneletPose lanelet_pose;
      lanelet_pose.lanelet_id = lane_change_parameters_->target.lanelet_id;
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
}  // namespace vehicle
}  // namespace entity_behavior
