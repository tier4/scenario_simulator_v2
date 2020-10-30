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

#include <simulation_api/behavior/vehicle/vehicle_action_node.hpp>

#include <string>
#include <memory>
#include <vector>
#include <utility>

namespace entity_behavior
{
VehicleActionNode::VehicleActionNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: ActionNode(name, config) {}

void VehicleActionNode::getBlackBoardValues()
{
  ActionNode::getBlackBoardValues();
  if (!getInput<std::shared_ptr<simulation_api::entity::VehicleParameters>>(
      "vehicle_parameters", vehicle_parameters))
  {
    throw BehaviorTreeRuntimeError("failed to get input vehicle_parameters in VehicleActionNode");
  }
}

boost::optional<double> VehicleActionNode::getDistanceToStopLine(
  const std::vector<int> & following_lanelets)
{
  return hdmap_utils->getDistanceToStopLine(following_lanelets, entity_status.lanelet_id,
           entity_status.s);
}

boost::optional<double> VehicleActionNode::getDistanceToFrontEntity()
{
  if (entity_status.coordinate != simulation_api::entity::CoordinateFrameTypes::LANE) {
    return boost::none;
  }
  auto status = getFrontEntityStatus();
  if (!status) {
    return boost::none;
  }
  if (status->coordinate != simulation_api::entity::CoordinateFrameTypes::LANE) {
    return boost::none;
  }
  return hdmap_utils->getLongitudinalDistance(entity_status.lanelet_id, entity_status.s,
           status->lanelet_id, status->s);
}

boost::optional<simulation_api::entity::EntityStatus> VehicleActionNode::getFrontEntityStatus()
{
  boost::optional<double> front_entity_distance, front_entity_speed;
  std::string front_entity_name = "";
  for (const auto & each : other_entity_status) {
    if (each.second.coordinate == simulation_api::entity::CoordinateFrameTypes::LANE) {
      auto distance = hdmap_utils->getLongitudinalDistance(entity_status.lanelet_id,
          entity_status.s,
          each.second.lanelet_id,
          each.second.s);
      if (distance) {
        if (distance.get() < 40) {
          if (!front_entity_distance && !front_entity_speed) {
            front_entity_speed = each.second.twist.linear.x;
            front_entity_distance = distance.get();
            front_entity_name = each.first;
          } else {
            if (front_entity_distance.get() < distance.get()) {
              front_entity_speed = each.second.twist.linear.x;
              front_entity_distance = distance.get();
              front_entity_name = each.first;
            }
          }
        }
      }
    }
  }
  if (!front_entity_distance && !front_entity_speed) {
    return boost::none;
  }
  return other_entity_status[front_entity_name];
}

boost::optional<double> VehicleActionNode::getDistanceToConflictingEntity(
  const std::vector<int> & following_lanelets) const
{
  auto conflicting_entity_status = getConflictingEntityStatus(following_lanelets);
  if (!conflicting_entity_status) {
    return boost::none;
  }
  std::vector<double> dists;
  std::vector<std::pair<int, double>> collision_points;
  for (const auto & lanelet_id : following_lanelets) {
    auto stop_position_s = hdmap_utils->getCollisionPointInLaneCoordinate(lanelet_id,
        conflicting_entity_status->lanelet_id);
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
  if (dists.size() != 0) {
    auto iter = std::max_element(dists.begin(), dists.end());
    size_t index = std::distance(dists.begin(), iter);
    double stop_s = collision_points[index].second;
    int stop_lanelet_id = collision_points[index].first;
    geometry_msgs::msg::Vector3 rpy;
    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Accel accel;
    simulation_api::entity::EntityStatus stop_target_status(0.0, stop_lanelet_id,
      stop_s, 0, rpy, twist, accel);
    auto dist_to_stop_target = hdmap_utils->getLongitudinalDistance(
      entity_status.lanelet_id, entity_status.s,
      stop_target_status.lanelet_id, stop_target_status.s);
    return dist_to_stop_target;
  }
  return boost::none;
}

boost::optional<simulation_api::entity::EntityStatus> VehicleActionNode::getConflictingEntityStatus(
  const std::vector<int> & following_lanelets) const
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(following_lanelets);
  std::vector<simulation_api::entity::EntityStatus> conflicting_entity_status;
  for (const auto & status : other_entity_status) {
    if (status.second.coordinate == simulation_api::entity::CoordinateFrameTypes::LANE) {
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
    simulation_api::entity::EntityStatus conflicting_entity_status(0.0, stop_lanelet_id,
      stop_s, 0, rpy, twist, accel);
    return conflicting_entity_status;
  }
  return boost::none;
}

bool VehicleActionNode::foundConflictingEntity(const std::vector<int> & following_lanelets) const
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(following_lanelets);
  for (const auto & status : other_entity_status) {
    if (status.second.coordinate == simulation_api::entity::CoordinateFrameTypes::LANE) {
      if (std::count(conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        status.second.lanelet_id) >= 1)
      {
        return true;
      }
    }
  }
  return false;
}

double VehicleActionNode::calculateStopDistance() const
{
  return std::pow(entity_status.twist.linear.x, 2) / (2 * 5);
}

simulation_api::entity::EntityStatus VehicleActionNode::calculateEntityStatusUpdated(
  double target_speed,
  const std::vector<int> & following_lanelets) const
{
  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::WORLD) {
    throw BehaviorTreeRuntimeError(
            "entity status must be lane when we call calculateEntityStatusUpdated function in VehicleActionNode");
  }
  geometry_msgs::msg::Accel accel_new;
  accel_new = entity_status.accel;
  double target_accel = (target_speed - entity_status.twist.linear.x) / step_time;
  if (entity_status.twist.linear.x > target_speed) {
    target_accel = boost::algorithm::clamp(target_accel, -5, 0);
  } else {
    target_accel = boost::algorithm::clamp(target_accel, 0, 3);
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
  int new_lanelet_id = entity_status.lanelet_id;
  double new_s = entity_status.s + (twist_new.linear.x + entity_status.twist.linear.x) / 2.0 *
    step_time;

  bool calculation_success = false;
  for (size_t i = 0; i < following_lanelets.size(); i++) {
    if (following_lanelets[i] == entity_status.lanelet_id) {
      double length = hdmap_utils->getLaneletLength(entity_status.lanelet_id);
      if (length < new_s) {
        calculation_success = true;
        if (i != (following_lanelets.size() - 1)) {
          new_s = new_s - length;
          new_lanelet_id = following_lanelets[i + 1];
          break;
        } else {
          new_s = new_s - length;
          auto next_ids = hdmap_utils->getNextLaneletIds(following_lanelets[i]);
          if (next_ids.size() == 0) {
            throw BehaviorTreeRuntimeError(
                    "failed to get next lane in calculateEntityStatusUpdated function in VehicleActionNode");
          }
          new_lanelet_id = next_ids[0];
          break;
        }
      }
    }
  }
  if (!calculation_success) {
    throw BehaviorTreeRuntimeError(
            "failed to calculate next status calculateEntityStatusUpdated function in VehicleActionNode");
  }
  simulation_api::entity::EntityStatus entity_status_updated(current_time + step_time,
    new_lanelet_id, new_s, entity_status.offset, entity_status.rpy, twist_new, accel_new);
  return entity_status_updated;
}

simulation_api::entity::EntityStatus VehicleActionNode::calculateEntityStatusUpdated(
  double target_speed) const
{
  geometry_msgs::msg::Accel accel_new;
  double target_accel = (target_speed - entity_status.twist.linear.x) / step_time;
  if (entity_status.twist.linear.x > target_speed) {
    target_accel = boost::algorithm::clamp(target_accel, -5, 0);
  } else {
    target_accel = boost::algorithm::clamp(target_accel, 0, 3);
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
  simulation_api::entity::EntityStatus entity_status_updated(current_time + step_time,
    entity_status.lanelet_id, new_s, entity_status.offset, entity_status.rpy, twist_new, accel_new);
  return entity_status_updated;
}
}  // namespace entity_behavior
