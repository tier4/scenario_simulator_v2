// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <simulation_controller/behavior/vehicle/behavior_tree.hpp>
#include <simulation_controller/behavior/vehicle/follow_lane_action.hpp>
#include <quaternion_operation/quaternion_operation.h>

#include <boost/algorithm/clamp.hpp>

#include <iostream>
#include <algorithm>
#include <string>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace entity_behavior
{
namespace vehicle
{
FollowLaneAction::FollowLaneAction(const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config) {}


void FollowLaneAction::decelerateInFrontOfConflictingEntity(
  const std::vector<int> & following_lanelets)
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(following_lanelets);
  std::vector<simulation_controller::entity::EntityStatus> conflicting_entity_status;
  for (const auto & status : other_entity_status) {
    if (status.second.coordinate == simulation_controller::entity::CoordinateFrameTypes::LANE) {
      if (std::count(conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        status.second.lanelet_id) >= 1)
      {
        conflicting_entity_status.push_back(status.second);
      }
    }
  }
  std::vector<double> dists;
  std::vector<std::pair<int, double>> collision_points;
  for (const auto & status : conflicting_entity_status) {
    for (const auto & lanelet_id : following_lanelets) {
      auto stop_position_s = hdmap_utils->getCollisionPointInLaneCoordinate(lanelet_id,
          status.lanelet_id);
      if (stop_position_s) {
        auto dist = hdmap_utils->getLongitudinalDistance(entity_status.lanelet_id,
            entity_status.s,
            lanelet_id, stop_position_s.get());
        if (dist) {
          dists.push_back(dist.get());
          collision_points.push_back(std::make_pair(lanelet_id, stop_position_s.get()));
        }
      }
    }
  }
  if (dists.size() != 0) {
    auto iter = std::max_element(dists.begin(), dists.end());
    size_t index = std::distance(dists.begin(), iter);
    double stop_s = collision_points[index].second;
    int stop_lanelet_id = collision_points[index].first;
    geometry_msgs::msg::Vector3 rpy;
    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Accel accel;
    simulation_controller::entity::EntityStatus stop_target_status(0.0, stop_lanelet_id,
      stop_s, 0, rpy, twist, accel);
    auto dist_to_stop_target = hdmap_utils->getLongitudinalDistance(
      entity_status.lanelet_id, entity_status.s,
      stop_target_status.lanelet_id, stop_target_status.s);
    if (dist_to_stop_target) {
      double rest_distance = dist_to_stop_target.get() -
        (vehicle_parameters->bounding_box.dimensions.length + 5);
      if (rest_distance < std::pow(entity_status.twist.linear.x, 2) / (2 * 5)) {
        if (rest_distance > 0) {
          target_speed = std::sqrt(2 * 5 * rest_distance);
        } else {
          target_speed = 0;
        }
      }
    }
  }
}

BT::NodeStatus FollowLaneAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }

  if (entity_status.coordinate == simulation_controller::entity::CoordinateFrameTypes::WORLD) {
    if (target_speed) {
      if (target_speed.get() > vehicle_parameters->performance.max_speed) {
        target_speed = vehicle_parameters->performance.max_speed;
      }
    } else {
      target_speed = vehicle_parameters->performance.max_speed;
    }
    double target_accel = (target_speed.get() - entity_status.twist.linear.x) / step_time;
    if (entity_status.twist.linear.x > target_speed.get()) {
      target_accel = boost::algorithm::clamp(target_accel, -5, 0);
      /* target_accel = boost::algorithm::clamp(target_accel,
        -1*vehicle_parameters->performance.max_deceleration, vehicle_parameters->performance.max_acceleration);
      */
    } else {
      target_accel = boost::algorithm::clamp(target_accel, 0, 3);
      /*
      target_accel = boost::algorithm::clamp(target_accel,
        -1*vehicle_parameters->performance.max_deceleration, vehicle_parameters->performance.max_acceleration);
      */
    }
    geometry_msgs::msg::Accel accel_new;
    accel_new = entity_status.accel;
    accel_new.linear.x = target_accel;

    geometry_msgs::msg::Twist twist_new;
    twist_new.linear.x = entity_status.twist.linear.x + entity_status.accel.linear.x * step_time;
    twist_new.linear.y = entity_status.twist.linear.y + entity_status.accel.linear.y * step_time;
    twist_new.linear.z = entity_status.twist.linear.z + entity_status.accel.linear.z * step_time;
    twist_new.angular.x = entity_status.twist.angular.x + entity_status.accel.angular.x * step_time;
    twist_new.angular.y = entity_status.twist.angular.y + entity_status.accel.angular.y * step_time;
    twist_new.angular.z = entity_status.twist.angular.z + entity_status.accel.angular.z * step_time;

    geometry_msgs::msg::Pose pose_new;
    geometry_msgs::msg::Vector3 angular_trans_vec;
    angular_trans_vec.z = twist_new.angular.z * step_time;
    geometry_msgs::msg::Quaternion angular_trans_quat =
      quaternion_operation::convertEulerAngleToQuaternion(angular_trans_vec);
    pose_new.orientation =
      quaternion_operation::rotation(entity_status.pose.orientation, angular_trans_quat);
    Eigen::Vector3d trans_vec;
    trans_vec(0) = twist_new.linear.x * step_time;
    trans_vec(1) = twist_new.linear.y * step_time;
    trans_vec(2) = 0;
    Eigen::Matrix3d rotation_mat = quaternion_operation::getRotationMatrix(pose_new.orientation);
    trans_vec = rotation_mat * trans_vec;
    pose_new.position.x = trans_vec(0) + entity_status.pose.position.x;
    pose_new.position.y = trans_vec(1) + entity_status.pose.position.y;
    pose_new.position.z = trans_vec(2) + entity_status.pose.position.z;

    simulation_controller::entity::EntityStatus entity_status_updated(current_time + step_time,
      pose_new, twist_new,
      accel_new);
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  }
  if (entity_status.coordinate == simulation_controller::entity::CoordinateFrameTypes::LANE) {
    for (const auto & each : other_entity_status) {
      if (each.second.coordinate == simulation_controller::entity::CoordinateFrameTypes::LANE) {
        auto distance = hdmap_utils->getLongitudinalDistance(entity_status.lanelet_id,
            entity_status.s,
            each.second.lanelet_id,
            each.second.s);
        if (distance) {
          if (distance.get() < 40) {
            target_speed = each.second.twist.linear.x;
          }
        }
      }
    }
    auto following_lanelets = hdmap_utils->getFollowingLanelets(entity_status.lanelet_id, 50);
    decelerateInFrontOfConflictingEntity(following_lanelets);
    if (!target_speed) {
      target_speed = hdmap_utils->getSpeedLimit(following_lanelets);
    }
    geometry_msgs::msg::Accel accel_new;
    accel_new = entity_status.accel;

    double target_accel = (target_speed.get() - entity_status.twist.linear.x) / step_time;
    if (entity_status.twist.linear.x > target_speed.get()) {
      target_accel = boost::algorithm::clamp(target_accel, -5, 0);
      /*
      target_accel = boost::algorithm::clamp(target_accel,
        -1*vehicle_parameters->performance.max_deceleration, vehicle_parameters->performance.max_acceleration);
      */
    } else {
      target_accel = boost::algorithm::clamp(target_accel, 0, 3);
      /*
      target_accel = boost::algorithm::clamp(target_accel,
        -1*vehicle_parameters->performance.max_deceleration, vehicle_parameters->performance.max_acceleration);
      */
    }
    accel_new.linear.x = target_accel;
    geometry_msgs::msg::Twist twist_new;
    twist_new.linear.x = boost::algorithm::clamp(
      entity_status.twist.linear.x + accel_new.linear.x * step_time,
      0, vehicle_parameters->performance.max_speed);
    twist_new.linear.y = 0.0;
    twist_new.linear.z = 0.0;
    twist_new.angular.x = 0.0;
    twist_new.angular.y = 0.0;
    twist_new.angular.z = 0.0;

    double new_s = entity_status.s + (twist_new.linear.x + entity_status.twist.linear.x) / 2.0 *
      step_time;
    geometry_msgs::msg::Vector3 rpy = entity_status.rpy;
    simulation_controller::entity::EntityStatus entity_status_updated(current_time + step_time,
      entity_status.lanelet_id, new_s, entity_status.offset, rpy, twist_new, accel_new);
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  }
}
}  // namespace vehicle
}  // namespace entity_behavior
