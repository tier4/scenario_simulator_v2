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

#include <simulation_api/behavior/action_node.hpp>

#include <string>
#include <memory>
#include <unordered_map>
#include <vector>
#include <utility>
#include <set>

namespace entity_behavior
{

ActionNode::ActionNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{}

BT::NodeStatus ActionNode::executeTick()
{
  return BT::ActionNodeBase::executeTick();
}

void ActionNode::getBlackBoardValues()
{
  if (!getInput("request", request)) {
    throw BehaviorTreeRuntimeError("failed to get input request in ActionNode");
  }
  if (!getInput<double>("step_time", step_time)) {
    throw BehaviorTreeRuntimeError("failed to get input step_time in ActionNode");
  }
  if (!getInput<double>("current_time", current_time)) {
    throw BehaviorTreeRuntimeError("failed to get input current_time in ActionNode");
  }
  if (!getInput<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils", hdmap_utils)) {
    throw BehaviorTreeRuntimeError("failed to get input hdmap_utils in ActionNode");
  }

  if (!getInput<openscenario_msgs::msg::EntityStatus>("entity_status", entity_status)) {
    throw BehaviorTreeRuntimeError("failed to get input entity_status in ActionNode");
  }

  if (!getInput<boost::optional<double>>("target_speed", target_speed)) {
    target_speed = boost::none;
  }

  if (!getInput<std::unordered_map<std::string,
    openscenario_msgs::msg::EntityStatus>>("other_entity_status", other_entity_status))
  {
    throw BehaviorTreeRuntimeError("failed to get input other_entity_status in ActionNode");
  }
  if (!getInput<std::unordered_map<std::string,
    openscenario_msgs::msg::EntityType>>("entity_type_list", entity_type_list))
  {
    throw BehaviorTreeRuntimeError("failed to get input entity_type_list in ActionNode");
  }
}

boost::optional<double> ActionNode::getYieldStopDistance(
  const std::vector<std::int64_t> & following_lanelets)
{
  std::set<double> dists;
  const auto lanelet_ids_list = hdmap_utils->getRightOfWayLaneletIds(following_lanelets);
  for (const auto & status : other_entity_status) {
    for (const auto & following_lanelet : following_lanelets) {
      for (const std::int64_t & lanelet_id : lanelet_ids_list.at(following_lanelet)) {
        if (lanelet_id == status.second.lanelet_pose.lanelet_id) {
          auto distance = hdmap_utils->getLongitudinalDistance(
            entity_status.lanelet_pose.lanelet_id,
            entity_status.lanelet_pose.s, following_lanelet, 0);
          if (distance) {
            dists.insert(distance.get());
          }
        }
      }
    }
  }
  if (dists.size() == 0) {
    return boost::none;
  }
  return *dists.begin();
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
  const auto lanelet_ids = hdmap_utils->getRightOfWayLaneletIds(
    entity_status.lanelet_pose.lanelet_id);
  if (lanelet_ids.size() == 0) {
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

boost::optional<double> ActionNode::getDistanceToStopLine(
  const std::vector<std::int64_t> & following_lanelets)
{
  return hdmap_utils->getDistanceToStopLine(following_lanelets, entity_status.lanelet_pose);
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
  boost::optional<double> front_entity_distance, front_entity_speed;
  std::string front_entity_name = "";
  for (const auto & each : other_entity_status) {
    if(entity_status.lanelet_pose_valid || each.second.lanelet_pose_valid) {
      continue;
    }
    auto distance = hdmap_utils->getLongitudinalDistance(
      entity_status.lanelet_pose,
      each.second.lanelet_pose);
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
  const std::vector<std::int64_t> & following_lanelets) const
{
  auto conflicting_entity_status = getConflictingEntityStatus(following_lanelets);
  if (!conflicting_entity_status) {
    return boost::none;
  }
  std::vector<double> dists;
  std::vector<std::pair<int, double>> collision_points;
  for (const auto & lanelet_id : following_lanelets) {
    auto stop_position_s = hdmap_utils->getCollisionPointInLaneCoordinate(lanelet_id,
        conflicting_entity_status->lanelet_pose.lanelet_id);
    if (stop_position_s) {
      auto dist = hdmap_utils->getLongitudinalDistance(entity_status.lanelet_pose.lanelet_id,
          entity_status.lanelet_pose.s,
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

boost::optional<openscenario_msgs::msg::EntityStatus> ActionNode::getConflictingEntityStatus(
  const std::vector<std::int64_t> & following_lanelets) const
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(following_lanelets);
  std::vector<openscenario_msgs::msg::EntityStatus> conflicting_entity_status;
  for (const auto & status : other_entity_status) {
    if (std::count(conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
      status.second.lanelet_pose.lanelet_id) >= 1)
    {
      conflicting_entity_status.push_back(status.second);
    }
  }
  std::vector<double> dists;
  std::vector<std::pair<int, double>> collision_points;
  for (const auto & status : conflicting_entity_status) {
    for (const auto & lanelet_id : following_lanelets) {
      auto stop_position_s = hdmap_utils->getCollisionPointInLaneCoordinate(lanelet_id,
          status.lanelet_pose.lanelet_id);
      if (stop_position_s) {
        auto dist = hdmap_utils->getLongitudinalDistance(entity_status.lanelet_pose.lanelet_id,
            entity_status.lanelet_pose.s,
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
    if (std::count(conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
      status.second.lanelet_pose.lanelet_id) >= 1)
    {
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
