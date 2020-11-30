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

#include <simulation_api/behavior/pedestrian/follow_lane_action.hpp>
#include <quaternion_operation/quaternion_operation.h>

#include <boost/algorithm/clamp.hpp>

#include <iostream>
#include <string>
#include <memory>
#include <vector>

namespace entity_behavior
{
namespace pedestrian
{
FollowLaneAction::FollowLaneAction(const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::PedestrianActionNode(name, config) {}

void FollowLaneAction::getBlackBoardValues()
{
  PedestrianActionNode::getBlackBoardValues();
}

BT::NodeStatus FollowLaneAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }

  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::WORLD) {
    geometry_msgs::msg::Accel accel_new;
    accel_new = entity_status.accel;

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

    simulation_api::entity::EntityStatus entity_status_updated(current_time + step_time,
      pose_new, twist_new,
      accel_new);
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  }
  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::LANE) {
    auto following_lanelets = hdmap_utils->getFollowingLanelets(entity_status.lanelet_id);
    if (!target_speed) {
      target_speed = hdmap_utils->getSpeedLimit(following_lanelets);
    }
    geometry_msgs::msg::Accel accel_new;
    accel_new = entity_status.accel;

    double target_accel = (target_speed.get() - entity_status.twist.linear.x) / step_time;
    if (entity_status.twist.linear.x > target_speed.get()) {
      target_accel = boost::algorithm::clamp(target_accel, -5, 0);
      /*　target_accel = boost::algorithm::clamp(target_accel,
        -1*vehicle_param_ptr->performance.max_deceleration, vehicle_param_ptr->performance.max_acceleration);
        */
    } else {
      target_accel = boost::algorithm::clamp(target_accel, 0, 3);
      /* target_accel = boost::algorithm::clamp(target_accel,
        -1*vehicle_param_ptr->performance.max_deceleration, vehicle_param_ptr->performance.max_acceleration);*/
    }
    accel_new.linear.x = target_accel;
    geometry_msgs::msg::Twist twist_new;
    twist_new.linear.x = boost::algorithm::clamp(
      entity_status.twist.linear.x + accel_new.linear.x * step_time,
      0, 5.0);
    twist_new.linear.y = 0.0;
    twist_new.linear.z = 0.0;
    twist_new.angular.x = 0.0;
    twist_new.angular.y = 0.0;
    twist_new.angular.z = 0.0;

    double new_s = entity_status.s + (twist_new.linear.x + entity_status.twist.linear.x) / 2.0 *
      step_time;
    geometry_msgs::msg::Vector3 rpy = entity_status.rpy;
    simulation_api::entity::EntityStatus entity_status_updated(current_time + step_time,
      entity_status.lanelet_id, new_s, entity_status.offset, rpy, twist_new, accel_new);
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::FAILURE;
}
}      // namespace pedestrian
}  // namespace entity_behavior
