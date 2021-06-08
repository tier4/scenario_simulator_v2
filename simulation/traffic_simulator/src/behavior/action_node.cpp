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

#include <algorithm>
#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/behavior/action_node.hpp>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
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
  if (!getInput<std::shared_ptr<traffic_simulator::TrafficLightManager>>(
        "traffic_light_manager", traffic_light_manager)) {
    THROW_SIMULATION_ERROR("failed to get input traffic_light_manager in ActionNode");
  }
  if (!getInput<openscenario_msgs::msg::EntityStatus>("entity_status", entity_status)) {
    THROW_SIMULATION_ERROR("failed to get input entity_status in ActionNode");
  }

  if (!getInput<boost::optional<double>>("target_speed", target_speed)) {
    target_speed = boost::none;
  }

  if (!getInput<std::unordered_map<std::string, openscenario_msgs::msg::EntityStatus>>(
        "other_entity_status", other_entity_status)) {
    THROW_SIMULATION_ERROR("failed to get input other_entity_status in ActionNode");
  }
  if (!getInput<std::unordered_map<std::string, openscenario_msgs::msg::EntityType>>(
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

openscenario_msgs::msg::EntityStatus ActionNode::stopAtEndOfRoad()
{
  openscenario_msgs::msg::EntityStatus entity_status_updated = entity_status;
  entity_status_updated.time = current_time + step_time;
  entity_status_updated.action_status.twist = geometry_msgs::msg::Twist();
  entity_status_updated.action_status.accel = geometry_msgs::msg::Accel();
  return entity_status_updated;
}

std::vector<openscenario_msgs::msg::EntityStatus> ActionNode::getOtherEntityStatus(
  std::int64_t lanelet_id)
{
  std::vector<openscenario_msgs::msg::EntityStatus> ret;
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
  std::set<double> dists;
  for (const auto & lanelet : following_lanelets) {
    const auto right_of_way_ids = hdmap_utils->getRightOfWayLaneletIds(lanelet);
    for (const auto right_of_way_id : right_of_way_ids) {
      const auto other_status = getOtherEntityStatus(right_of_way_id);
      if (other_status.size() != 0) {
        auto distance = hdmap_utils->getLongitudinalDistance(
          entity_status.lanelet_pose.lanelet_id, entity_status.lanelet_pose.s, lanelet, 0);
        if (distance) {
          dists.insert(distance.get());
        }
      }
    }
    if (dists.size() != 0) {
      return *dists.begin();
    }
  }
  return boost::none;
}

std::vector<openscenario_msgs::msg::EntityStatus> ActionNode::getRightOfWayEntities(
  const std::vector<std::int64_t> & following_lanelets)
{
  std::vector<openscenario_msgs::msg::EntityStatus> ret;
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

std::vector<openscenario_msgs::msg::EntityStatus> ActionNode::getRightOfWayEntities()
{
  std::vector<openscenario_msgs::msg::EntityStatus> ret;
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
  const std::vector<geometry_msgs::msg::Point> & waypoints)
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
      const auto collision_point = hdmap_utils->getDistanceToTrafficLightStopLine(waypoints, id);
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

boost::optional<double> ActionNode::getDistanceToFrontEntity()
{
  auto status = getFrontEntityStatus();
  if (!status) {
    return boost::none;
  }
  return hdmap_utils->getLongitudinalDistance(entity_status.lanelet_pose, status->lanelet_pose);
}

boost::optional<openscenario_msgs::msg::EntityStatus> ActionNode::getFrontEntityStatus()
{
  traffic_simulator::helper::StopWatch<std::chrono::milliseconds> stop_watch(
    "getFrontEntityStatus");
  boost::optional<double> front_entity_distance, front_entity_speed;
  std::string front_entity_name = "";
  for (const auto & each : other_entity_status) {
    if (!entity_status.lanelet_pose_valid || !each.second.lanelet_pose_valid) {
      continue;
    }
    stop_watch.start();
    boost::optional<double> distance =
      hdmap_utils->getLongitudinalDistance(entity_status.lanelet_pose, each.second.lanelet_pose);
    stop_watch.stop();
    stop_watch.print();
    if (distance) {
      if (distance.get() < 40) {
        if (!front_entity_distance && !front_entity_speed) {
          front_entity_speed = each.second.action_status.twist.linear.x;
          front_entity_distance = distance.get();
          front_entity_name = each.first;
        } else {
          if (front_entity_distance.get() > distance.get()) {
            front_entity_speed = each.second.action_status.twist.linear.x;
            front_entity_distance = distance.get();
            front_entity_name = each.first;
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

boost::optional<double> ActionNode::getDistanceToConflictingEntity(
  const std::vector<std::int64_t> & route_lanelets,
  const traffic_simulator::math::CatmullRomSpline & spline)
{
  auto conflicting_entity_status = getConflictingEntityStatusOnRoute(route_lanelets);
  if (conflicting_entity_status.empty()) {
    return boost::none;
  }
  std::set<double> distances;
  for (const auto status : conflicting_entity_status) {
    if (status.lanelet_pose_valid) {
      auto polygon = hdmap_utils->getLaneletPolygon(status.lanelet_pose.lanelet_id);
      auto s = spline.getCollisionPointIn2D(polygon);
      if (s) {
        distances.insert(s.get());
      }
    }
  }
  if (distances.empty()) {
    return boost::none;
  }
  return *distances.begin();
}

boost::optional<double> ActionNode::getDistanceToConflictingEntity(
  const std::vector<std::int64_t> & following_lanelets) const
{
  auto conflicting_entity_status = getConflictingEntityStatus(following_lanelets);
  if (!conflicting_entity_status) {
    return boost::none;
  }
  std::vector<double> dists;
  std::vector<std::pair<int, double>> collision_points;
  for (const auto & lanelet_id : following_lanelets) {
    auto stop_position_s = hdmap_utils->getCollisionPointInLaneCoordinate(
      lanelet_id, conflicting_entity_status->lanelet_pose.lanelet_id);
    if (stop_position_s) {
      auto dist = hdmap_utils->getLongitudinalDistance(
        entity_status.lanelet_pose.lanelet_id, entity_status.lanelet_pose.s, lanelet_id,
        stop_position_s.get());
      if (dist) {
        dists.push_back(dist.get());
        collision_points.push_back(std::make_pair(lanelet_id, stop_position_s.get()));
      }
    }
  }
  if (dists.size() != 0) {
    auto iter = std::min_element(dists.begin(), dists.end());
    size_t index = std::distance(dists.begin(), iter);
    double stop_s = collision_points[index].second;
    std::int64_t stop_lanelet_id = collision_points[index].first;
    geometry_msgs::msg::Vector3 rpy;
    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Accel accel;
    openscenario_msgs::msg::EntityStatus stop_target_status;
    stop_target_status.lanelet_pose.lanelet_id = stop_lanelet_id;
    stop_target_status.lanelet_pose.s = stop_s;
    stop_target_status.lanelet_pose.rpy = rpy;
    stop_target_status.action_status.twist = twist;
    stop_target_status.action_status.accel = accel;
    auto dist_to_stop_target = hdmap_utils->getLongitudinalDistance(
      entity_status.lanelet_pose.lanelet_id, entity_status.lanelet_pose.s,
      stop_target_status.lanelet_pose.lanelet_id, stop_target_status.lanelet_pose.s);
    return dist_to_stop_target;
  }
  return boost::none;
}

std::vector<openscenario_msgs::msg::EntityStatus> ActionNode::getConflictingEntityStatusOnRoute(
  const std::vector<std::int64_t> & route_lanelets) const
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(route_lanelets);
  std::vector<openscenario_msgs::msg::EntityStatus> conflicting_entity_status;
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

boost::optional<openscenario_msgs::msg::EntityStatus> ActionNode::getConflictingEntityStatus(
  const std::vector<std::int64_t> & following_lanelets) const
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(following_lanelets);
  std::vector<openscenario_msgs::msg::EntityStatus> conflicting_entity_status;
  for (const auto & status : other_entity_status) {
    if (
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        status.second.lanelet_pose.lanelet_id) >= 1) {
      conflicting_entity_status.push_back(status.second);
    }
  }
  std::vector<double> dists;
  std::vector<std::pair<int, double>> collision_points;
  for (const auto & status : conflicting_entity_status) {
    for (const auto & lanelet_id : following_lanelets) {
      auto stop_position_s =
        hdmap_utils->getCollisionPointInLaneCoordinate(lanelet_id, status.lanelet_pose.lanelet_id);
      if (stop_position_s) {
        auto dist = hdmap_utils->getLongitudinalDistance(
          entity_status.lanelet_pose.lanelet_id, entity_status.lanelet_pose.s, lanelet_id,
          stop_position_s.get());
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
    std::int64_t stop_lanelet_id = collision_points[index].first;
    geometry_msgs::msg::Vector3 rpy;
    geometry_msgs::msg::Twist twist;
    geometry_msgs::msg::Accel accel;
    openscenario_msgs::msg::EntityStatus conflicting_entity_status;
    conflicting_entity_status.lanelet_pose.lanelet_id = stop_lanelet_id;
    conflicting_entity_status.lanelet_pose.s = stop_s;
    conflicting_entity_status.lanelet_pose.offset = 0;
    conflicting_entity_status.lanelet_pose.rpy = rpy;
    conflicting_entity_status.action_status.twist = twist;
    conflicting_entity_status.action_status.accel = accel;
    conflicting_entity_status.pose =
      hdmap_utils->toMapPose(conflicting_entity_status.lanelet_pose).pose;
    return conflicting_entity_status;
  }
  return boost::none;
}

bool ActionNode::foundConflictingEntity(const std::vector<std::int64_t> & following_lanelets) const
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(following_lanelets);
  for (const auto & status : other_entity_status) {
    if (
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        status.second.lanelet_pose.lanelet_id) >= 1) {
      return true;
    }
  }
  return false;
}

double ActionNode::calculateStopDistance() const
{
  return std::pow(entity_status.action_status.twist.linear.x, 2) / (2 * 5);
}
}  // namespace entity_behavior
