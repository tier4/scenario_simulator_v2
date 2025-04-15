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
#include <behavior_tree_plugin/pedestrian/follow_lane_action.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace entity_behavior
{
namespace pedestrian
{
FollowLaneAction::FollowLaneAction(const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::PedestrianActionNode(name, config)
{
  auto parameterToSeeAroundMode = [](const std::string & parameter) {
    if (parameter == "ignore")
      return SeeAroundMode::ignore;
    else if (parameter == "respect")
      return SeeAroundMode::respect;
    THROW_SIMULATION_ERROR("Unknown see_around mode. It must be \"ignore\" or \"respect\".");
  };

  should_respect_see_around = parameterToSeeAroundMode(
    common::getParameter<std::string>("ignore_see_around_for_pedestrian", "ignore"));
}

void FollowLaneAction::getBlackBoardValues() { PedestrianActionNode::getBlackBoardValues(); }

DetectorStatus FollowLaneAction::detectObstacleInLane(
  const lanelet::Ids & pedestrian_lanes, const bool & see_around) const
{
  if (should_respect_see_around == SeeAroundMode::ignore) {
    return DetectorStatus::not_detected;
  }

  if (!see_around) {
    return DetectorStatus::not_detected;
  }

  auto hasObstacleInPedestrianLanes = [this](const lanelet::Ids & pedestrian_lanes_local) {
    lanelet::Ids other_entity_lane_ids;
    for (const auto & [_, status] : other_entity_status) {
      if (status.isInLanelet()) {
        other_entity_lane_ids.push_back(status.getLaneletId());
      }
    }
    std::unordered_set<lanelet::Id> other_lane_id_set(
      other_entity_lane_ids.begin(), other_entity_lane_ids.end());
    for (const auto & pedestrian_lane : pedestrian_lanes_local) {
      if (other_lane_id_set.count(pedestrian_lane)) {
        return DetectorStatus::detected;
      }
    }
    return DetectorStatus::not_detected;
  };

  auto hasObstacleInFrontOfPedestrian = [this]() {
    using math::geometry::operator-;

    const auto & pedestrian_position = canonicalized_entity_status->getMapPose().position;

    for (const auto & [_, entity_status] : other_entity_status) {
      const auto & other_position = entity_status.getMapPose().position;
      const auto relative_position = other_position - pedestrian_position;
      const double relative_angle_rad = std::atan2(relative_position.y, relative_position.x);
      if (relative_angle_rad > 0) {
        return DetectorStatus::detected;
      }
    }
    return DetectorStatus::not_detected;
  };

  if (
    hasObstacleInPedestrianLanes(pedestrian_lanes) == DetectorStatus::detected &&
    hasObstacleInFrontOfPedestrian() == DetectorStatus::detected) {
    return DetectorStatus::detected;
  }

  return DetectorStatus::not_detected;
}

BT::NodeStatus FollowLaneAction::tick()
{
  getBlackBoardValues();
  if (
    request != traffic_simulator::behavior::Request::NONE &&
    request != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return BT::NodeStatus::FAILURE;
  }
  if (!canonicalized_entity_status->isInLanelet()) {
    stopEntity();
    return BT::NodeStatus::RUNNING;
  }
  auto following_lanelets =
    hdmap_utils->getFollowingLanelets(canonicalized_entity_status->getLaneletId());
  if (!target_speed) {
    target_speed = hdmap_utils->getSpeedLimit(following_lanelets);
  }

  const auto obstacle_detector_result =
    detectObstacleInLane(following_lanelets, behavior_parameter.see_around);
  target_speed = (obstacle_detector_result == DetectorStatus::detected) ? 0.0 : target_speed;

  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed.value()));
  return BT::NodeStatus::RUNNING;
}
}  // namespace pedestrian
}  // namespace entity_behavior
