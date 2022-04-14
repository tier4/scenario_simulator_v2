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

#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <behavior_tree_plugin/action_node.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/math/bounding_box.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace entity_behavior
{
ActionNode::ActionNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

BT::NodeStatus ActionNode::executeTick() { return BT::ActionNodeBase::executeTick(); }

void ActionNode::getBlackBoardValues()
{
  if (!getInput("request", request)) {
    THROW_SIMULATION_ERROR("failed to get input request in ActionNode");
  }
  if (!getInput<double>("step_time", step_time)) {
    THROW_SIMULATION_ERROR("failed to get input step_time in ActionNode");
  }
  if (!getInput<double>("current_time", current_time)) {
    THROW_SIMULATION_ERROR("failed to get input current_time in ActionNode");
  }
  if (!getInput<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils", hdmap_utils)) {
    THROW_SIMULATION_ERROR("failed to get input hdmap_utils in ActionNode");
  }
  if (!getInput<std::shared_ptr<traffic_simulator::TrafficLightManagerBase>>(
        "traffic_light_manager", traffic_light_manager)) {
    THROW_SIMULATION_ERROR("failed to get input traffic_light_manager in ActionNode");
  }
  if (!getInput<traffic_simulator_msgs::msg::EntityStatus>("entity_status", entity_status)) {
    THROW_SIMULATION_ERROR("failed to get input entity_status in ActionNode");
  }

  if (!getInput<boost::optional<double>>("target_speed", target_speed)) {
    target_speed = boost::none;
  }

  if (!getInput<std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus>>(
        "other_entity_status", other_entity_status)) {
    THROW_SIMULATION_ERROR("failed to get input other_entity_status in ActionNode");
  }
  if (!getInput<std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType>>(
        "entity_type_list", entity_type_list)) {
    THROW_SIMULATION_ERROR("failed to get input entity_type_list in ActionNode");
  }
  if (!getInput<std::vector<std::int64_t>>("route_lanelets", route_lanelets)) {
    THROW_SIMULATION_ERROR("failed to get input route_lanelets in ActionNode");
  }
}

double ActionNode::getHorizon() const
{
  return boost::algorithm::clamp(entity_status.action_status.twist.linear.x * 5, 20, 50);
}

traffic_simulator_msgs::msg::EntityStatus ActionNode::stopAtEndOfRoad()
{
  traffic_simulator_msgs::msg::EntityStatus entity_status_updated = entity_status;
  entity_status_updated.time = current_time + step_time;
  entity_status_updated.action_status.twist = geometry_msgs::msg::Twist();
  entity_status_updated.action_status.accel = geometry_msgs::msg::Accel();
  return entity_status_updated;
}

std::vector<traffic_simulator_msgs::msg::EntityStatus> ActionNode::getOtherEntityStatus(
  std::int64_t lanelet_id)
{
  std::vector<traffic_simulator_msgs::msg::EntityStatus> ret;
  for (const auto & status : other_entity_status) {
    if (status.second.lanelet_pose_valid) {
      if (status.second.lanelet_pose.lanelet_id == lanelet_id) {
        ret.emplace_back(status.second);
      }
    }
  }
  return ret;
}

boost::optional<double> ActionNode::getYieldStopDistance(
  const std::vector<std::int64_t> & following_lanelets)
{
  std::set<double> distances;
  for (const auto & lanelet : following_lanelets) {
    const auto right_of_way_ids = hdmap_utils->getRightOfWayLaneletIds(lanelet);
    for (const auto right_of_way_id : right_of_way_ids) {
      const auto other_status = getOtherEntityStatus(right_of_way_id);
      if (other_status.size() != 0) {
        auto distance = hdmap_utils->getLongitudinalDistance(
          entity_status.lanelet_pose.lanelet_id, entity_status.lanelet_pose.s, lanelet, 0);
        if (distance) {
          distances.insert(distance.get());
        }
      }
    }
    if (distances.size() != 0) {
      return *distances.begin();
    }
  }
  return boost::none;
}

std::vector<traffic_simulator_msgs::msg::EntityStatus> ActionNode::getRightOfWayEntities(
  const std::vector<std::int64_t> & following_lanelets)
{
  std::vector<traffic_simulator_msgs::msg::EntityStatus> ret;
  const auto lanelet_ids_list = hdmap_utils->getRightOfWayLaneletIds(following_lanelets);
  for (const auto & status : other_entity_status) {
    for (const auto & following_lanelet : following_lanelets) {
      for (const std::int64_t & lanelet_id : lanelet_ids_list.at(following_lanelet)) {
        if (lanelet_id == status.second.lanelet_pose.lanelet_id) {
          ret.emplace_back(status.second);
        }
      }
    }
  }
  return ret;
}

std::vector<traffic_simulator_msgs::msg::EntityStatus> ActionNode::getRightOfWayEntities()
{
  std::vector<traffic_simulator_msgs::msg::EntityStatus> ret;
  const auto lanelet_ids =
    hdmap_utils->getRightOfWayLaneletIds(entity_status.lanelet_pose.lanelet_id);
  if (lanelet_ids.empty()) {
    return ret;
  }
  for (const auto & status : other_entity_status) {
    for (const std::int64_t & lanelet_id : lanelet_ids) {
      if (lanelet_id == status.second.lanelet_pose.lanelet_id) {
        ret.emplace_back(status.second);
      }
    }
  }
  return ret;
}

boost::optional<double> ActionNode::getDistanceToTrafficLightStopLine(
  const std::vector<std::int64_t> & route_lanelets,
  const traffic_simulator::math::CatmullRomInterface & spline)
{
  const auto traffic_light_ids = hdmap_utils->getTrafficLightIdsOnPath(route_lanelets);
  if (traffic_light_ids.empty()) {
    return boost::none;
  }
  std::set<double> collision_points = {};
  for (const auto id : traffic_light_ids) {
    const auto color = traffic_light_manager->getColor(id);
    if (
      color == traffic_simulator::TrafficLightColor::RED ||
      color == traffic_simulator::TrafficLightColor::YELLOW) {
      const auto collision_point = hdmap_utils->getDistanceToTrafficLightStopLine(spline, id);
      if (collision_point) {
        collision_points.insert(collision_point.get());
      }
    }
  }
  if (collision_points.empty()) {
    return boost::none;
  }
  return *collision_points.begin();
}

boost::optional<double> ActionNode::getDistanceToStopLine(
  const std::vector<std::int64_t> & route_lanelets,
  const std::vector<geometry_msgs::msg::Point> & waypoints)
{
  return hdmap_utils->getDistanceToStopLine(route_lanelets, waypoints);
}

boost::optional<double> ActionNode::getDistanceToFrontEntity(
  const traffic_simulator::math::CatmullRomInterface & spline)
{
  auto name = getFrontEntityName(spline);
  if (!name) {
    return boost::none;
  }
  return getDistanceToTargetEntityPolygon(spline, name.get());
}

boost::optional<std::string> ActionNode::getFrontEntityName(
  const traffic_simulator::math::CatmullRomInterface & spline)
{
  std::vector<double> distances;
  std::vector<std::string> entities;
  for (const auto & each : other_entity_status) {
    const auto distance = getDistanceToTargetEntityPolygon(spline, each.first);
    const auto quat = quaternion_operation::getRotation(
      entity_status.pose.orientation, other_entity_status.at(each.first).pose.orientation);
    /**
     * @note hard-coded parameter, if the Yaw value of RPY is in ~1.5708 -> 1.5708, entity is a candidate of front entity.
     */
    if (
      std::fabs(quaternion_operation::convertQuaternionToEulerAngle(quat).z) <=
      boost::math::constants::half_pi<double>()) {
      if (distance && distance.get() < 40) {
        entities.emplace_back(each.first);
        distances.emplace_back(distance.get());
      }
    }
  }
  if (entities.size() != distances.size()) {
    THROW_SIMULATION_ERROR("size of entities and distances vector does not match.");
  }
  if (distances.empty()) {
    return boost::none;
  }
  std::vector<double>::iterator iter = std::min_element(distances.begin(), distances.end());
  size_t index = std::distance(distances.begin(), iter);
  return entities[index];
}

boost::optional<double> ActionNode::getDistanceToTargetEntityOnCrosswalk(
  const traffic_simulator::math::CatmullRomInterface & spline,
  const traffic_simulator_msgs::msg::EntityStatus & status)
{
  if (status.lanelet_pose_valid) {
    auto polygon = hdmap_utils->getLaneletPolygon(status.lanelet_pose.lanelet_id);
    return spline.getCollisionPointIn2D(polygon, false, true);
  }
  return boost::none;
}

traffic_simulator_msgs::msg::EntityStatus ActionNode::getEntityStatus(
  const std::string target_name) const
{
  if (other_entity_status.find(target_name) != other_entity_status.end()) {
    return other_entity_status.at(target_name);
  }
  THROW_SIMULATION_ERROR("other entity : ", target_name, " does not exist.");
}

boost::optional<double> ActionNode::getDistanceToTargetEntityPolygon(
  const traffic_simulator::math::CatmullRomInterface & spline, const std::string target_name,
  double width_extension_right, double width_extension_left, double length_extension_front,
  double length_extension_rear)
{
  const auto status = getEntityStatus(target_name);
  if (status.lanelet_pose_valid == true) {
    return getDistanceToTargetEntityPolygon(
      spline, status, width_extension_right, width_extension_left, length_extension_front,
      length_extension_rear);
  }
  return boost::none;
}

boost::optional<double> ActionNode::getDistanceToTargetEntityPolygon(
  const traffic_simulator::math::CatmullRomInterface & spline,
  const traffic_simulator_msgs::msg::EntityStatus & status, double width_extension_right,
  double width_extension_left, double length_extension_front, double length_extension_rear)
{
  if (status.lanelet_pose_valid) {
    const auto polygon = traffic_simulator::math::transformPoints(
      status.pose, traffic_simulator::math::getPointsFromBbox(
                     status.bounding_box, width_extension_right, width_extension_left,
                     length_extension_front, length_extension_rear));
    return spline.getCollisionPointIn2D(polygon, false, true);
  }
  return boost::none;
}

boost::optional<double> ActionNode::getDistanceToConflictingEntity(
  const std::vector<std::int64_t> & route_lanelets,
  const traffic_simulator::math::CatmullRomInterface & spline)
{
  auto crosswalk_entity_status = getConflictingEntityStatusOnCrossWalk(route_lanelets);
  auto lane_entity_status = getConflictingEntityStatusOnLane(route_lanelets);
  std::set<double> distances;
  for (const auto & status : crosswalk_entity_status) {
    const auto s = getDistanceToTargetEntityOnCrosswalk(spline, status);
    if (s) {
      distances.insert(s.get());
    }
  }
  for (const auto & status : lane_entity_status) {
    const auto s = getDistanceToTargetEntityPolygon(spline, status, 0.0, 0.0, 0.0, 1.0);
    if (s) {
      distances.insert(s.get());
    }
  }
  if (distances.empty()) {
    return boost::none;
  }
  return *distances.begin();
}

std::vector<traffic_simulator_msgs::msg::EntityStatus>
ActionNode::getConflictingEntityStatusOnCrossWalk(
  const std::vector<std::int64_t> & route_lanelets) const
{
  std::vector<traffic_simulator_msgs::msg::EntityStatus> conflicting_entity_status;
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(route_lanelets);
  for (const auto & status : other_entity_status) {
    if (
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        status.second.lanelet_pose.lanelet_id) >= 1) {
      conflicting_entity_status.push_back(status.second);
    }
  }
  return conflicting_entity_status;
}

std::vector<traffic_simulator_msgs::msg::EntityStatus> ActionNode::getConflictingEntityStatusOnLane(
  const std::vector<std::int64_t> & route_lanelets) const
{
  std::vector<traffic_simulator_msgs::msg::EntityStatus> conflicting_entity_status;
  auto conflicting_lanes = hdmap_utils->getConflictingLaneIds(route_lanelets);
  for (const auto & status : other_entity_status) {
    if (
      std::count(
        conflicting_lanes.begin(), conflicting_lanes.end(),
        status.second.lanelet_pose.lanelet_id) >= 1) {
      conflicting_entity_status.push_back(status.second);
    }
  }
  return conflicting_entity_status;
}

bool ActionNode::foundConflictingEntity(const std::vector<std::int64_t> & following_lanelets) const
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(following_lanelets);
  auto conflicting_lanes = hdmap_utils->getConflictingLaneIds(following_lanelets);
  for (const auto & status : other_entity_status) {
    if (
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        status.second.lanelet_pose.lanelet_id) >= 1) {
      return true;
    }
    if (
      std::count(
        conflicting_lanes.begin(), conflicting_lanes.end(),
        status.second.lanelet_pose.lanelet_id) >= 1) {
      return true;
    }
  }
  return false;
}

double ActionNode::calculateStopDistance(double deceleration) const
{
  return (entity_status.action_status.twist.linear.x * entity_status.action_status.twist.linear.x) /
         (2 * std::fabs(deceleration));
}
}  // namespace entity_behavior
