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
  if (!getInput<traffic_simulator_msgs::msg::BehaviorParameter>(
        "behavior_parameter", behavior_parameter)) {
    behavior_parameter = traffic_simulator_msgs::msg::BehaviorParameter();
  }
  if (!getInput<traffic_simulator_msgs::msg::VehicleParameters>(
        "vehicle_parameters", vehicle_parameters)) {
    THROW_SIMULATION_ERROR("failed to get input vehicle_parameters in VehicleActionNode");
  }
  if (!getInput<std::shared_ptr<math::geometry::CatmullRomSpline>>(
        "reference_trajectory", reference_trajectory)) {
    THROW_SIMULATION_ERROR("failed to get input reference_trajectory in VehicleActionNode");
  }
}

traffic_simulator_msgs::msg::EntityStatus VehicleActionNode::calculateEntityStatusUpdated(
  double target_speed)
{
  geometry_msgs::msg::Accel accel_new;
  accel_new = entity_status.action_status.accel;
  double target_accel = (target_speed - entity_status.action_status.twist.linear.x) / step_time;
  if (entity_status.action_status.twist.linear.x > target_speed) {
    target_accel = boost::algorithm::clamp(
      target_accel, behavior_parameter.dynamic_constraints.max_deceleration * -1, 0);
  } else {
    target_accel = boost::algorithm::clamp(
      target_accel, 0, behavior_parameter.dynamic_constraints.max_acceleration);
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
              return stopAtEndOfRoad();
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
  return ActionNode::calculateEntityStatusUpdatedInWorldFrame(
    target_speed, behavior_parameter.dynamic_constraints);
}
}  // namespace entity_behavior
