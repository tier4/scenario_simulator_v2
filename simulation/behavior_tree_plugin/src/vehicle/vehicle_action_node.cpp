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

#include <behavior_tree_plugin/vehicle/vehicle_action_node.hpp>
#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <utility>
#include <vector>

namespace entity_behavior
{
VehicleActionNode::VehicleActionNode(const std::string & name, const BT::NodeConfiguration & config)
: ActionNode(name, config)
{
}

void VehicleActionNode::getBlackBoardValues()
{
  ActionNode::getBlackBoardValues();
  if (!getInput<traffic_simulator_msgs::msg::DriverModel>("driver_model", driver_model)) {
    driver_model = traffic_simulator_msgs::msg::DriverModel();
  }
  if (!getInput<traffic_simulator_msgs::msg::VehicleParameters>(
        "vehicle_parameters", vehicle_parameters)) {
    THROW_SIMULATION_ERROR("failed to get input vehicle_parameters in VehicleActionNode");
  }
}

traffic_simulator_msgs::msg::EntityStatus VehicleActionNode::calculateEntityStatusUpdated(
  double target_speed)
{
  geometry_msgs::msg::Accel accel_new;
  accel_new = entity_status.action_status.accel;
  double target_accel = (target_speed - entity_status.action_status.twist.linear.x) / step_time;
  if (entity_status.action_status.twist.linear.x > target_speed) {
    target_accel = boost::algorithm::clamp(target_accel, driver_model.deceleration * -1, 0);
  } else {
    target_accel = boost::algorithm::clamp(target_accel, 0, driver_model.acceleration);
  }
  accel_new.linear.x = target_accel;
  geometry_msgs::msg::Twist twist_new;
  twist_new.linear.x = boost::algorithm::clamp(
    entity_status.action_status.twist.linear.x + accel_new.linear.x * step_time, -10,
    vehicle_parameters.performance.max_speed);
  twist_new.linear.y = 0.0;
  twist_new.linear.z = 0.0;
  twist_new.angular.x = 0.0;
  twist_new.angular.y = 0.0;
  twist_new.angular.z = 0.0;
  std::int64_t new_lanelet_id = entity_status.lanelet_pose.lanelet_id;
  double new_s =
    entity_status.lanelet_pose.s +
    (twist_new.linear.x + entity_status.action_status.twist.linear.x) / 2.0 * step_time;
  if (new_s < 0) {
    auto previous_lanelet_ids =
      hdmap_utils->getPreviousLaneletIds(entity_status.lanelet_pose.lanelet_id);
    new_lanelet_id = previous_lanelet_ids[0];
    new_s = new_s + hdmap_utils->getLaneletLength(new_lanelet_id) - 0.01;
    traffic_simulator_msgs::msg::EntityStatus entity_status_updated;
    entity_status_updated.time = current_time + step_time;
    entity_status_updated.lanelet_pose.lanelet_id = new_lanelet_id;
    entity_status_updated.lanelet_pose.s = new_s;
    entity_status_updated.lanelet_pose.offset = entity_status.lanelet_pose.offset;
    entity_status_updated.lanelet_pose.rpy = entity_status.lanelet_pose.rpy;
    entity_status_updated.action_status.twist = twist_new;
    entity_status_updated.action_status.accel = accel_new;
    entity_status_updated.pose = hdmap_utils->toMapPose(entity_status_updated.lanelet_pose).pose;
    return entity_status_updated;
  } else {
    bool calculation_success = false;
    for (size_t i = 0; i < route_lanelets.size(); i++) {
      if (route_lanelets[i] == entity_status.lanelet_pose.lanelet_id) {
        double length = hdmap_utils->getLaneletLength(entity_status.lanelet_pose.lanelet_id);
        calculation_success = true;
        if (length < new_s) {
          if (i != (route_lanelets.size() - 1)) {
            new_s = new_s - length;
            new_lanelet_id = route_lanelets[i + 1];
            break;
          } else {
            new_s = new_s - length;
            auto next_ids = hdmap_utils->getNextLaneletIds(route_lanelets[i]);
            if (next_ids.empty()) {
              const auto ret = stopAtEndOfRoad();
              return ret;
            }
            new_lanelet_id = next_ids[0];
            break;
          }
        }
      }
    }
    if (!calculation_success) {
      THROW_SIMULATION_ERROR(
        "failed to calculate next status calculateEntityStatusUpdated function");
    }
    traffic_simulator_msgs::msg::EntityStatus entity_status_updated;
    entity_status_updated.time = current_time + step_time;
    entity_status_updated.lanelet_pose.lanelet_id = new_lanelet_id;
    entity_status_updated.lanelet_pose.s = new_s;
    entity_status_updated.lanelet_pose.offset = entity_status.lanelet_pose.offset;
    entity_status_updated.lanelet_pose.rpy = entity_status.lanelet_pose.rpy;
    entity_status_updated.pose = hdmap_utils->toMapPose(entity_status_updated.lanelet_pose).pose;
    entity_status_updated.action_status.twist = twist_new;
    entity_status_updated.action_status.accel = accel_new;
    return entity_status_updated;
  }
  THROW_SIMULATION_ERROR("failed to calculate next status calculateEntityStatusUpdated function");
}

traffic_simulator_msgs::msg::EntityStatus
VehicleActionNode::calculateEntityStatusUpdatedInWorldFrame(double target_speed)
{
  if (target_speed > vehicle_parameters.performance.max_speed) {
    target_speed = vehicle_parameters.performance.max_speed;
  } else {
    target_speed = entity_status.action_status.twist.linear.x;
  }
  double target_accel = (target_speed - entity_status.action_status.twist.linear.x) / step_time;
  if (entity_status.action_status.twist.linear.x > target_speed) {
    target_accel = boost::algorithm::clamp(target_accel, -1 * driver_model.deceleration, 0);
  } else {
    target_accel = boost::algorithm::clamp(target_accel, 0, driver_model.acceleration);
  }
  geometry_msgs::msg::Accel accel_new;
  accel_new = entity_status.action_status.accel;
  accel_new.linear.x = target_accel;

  geometry_msgs::msg::Twist twist_new;
  twist_new.linear.x = entity_status.action_status.twist.linear.x +
                       entity_status.action_status.accel.linear.x * step_time;
  twist_new.linear.y = entity_status.action_status.twist.linear.y +
                       entity_status.action_status.accel.linear.y * step_time;
  twist_new.linear.z = entity_status.action_status.twist.linear.z +
                       entity_status.action_status.accel.linear.z * step_time;
  twist_new.angular.x = entity_status.action_status.twist.angular.x +
                        entity_status.action_status.accel.angular.x * step_time;
  twist_new.angular.y = entity_status.action_status.twist.angular.y +
                        entity_status.action_status.accel.angular.y * step_time;
  twist_new.angular.z = entity_status.action_status.twist.angular.z +
                        entity_status.action_status.accel.angular.z * step_time;

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
  traffic_simulator_msgs::msg::EntityStatus entity_status_updated;
  entity_status_updated.time = current_time + step_time;
  entity_status_updated.pose = pose_new;
  entity_status_updated.action_status.twist = twist_new;
  entity_status_updated.action_status.accel = accel_new;
  entity_status_updated.lanelet_pose_valid = false;
  return entity_status_updated;
}
}  // namespace entity_behavior
