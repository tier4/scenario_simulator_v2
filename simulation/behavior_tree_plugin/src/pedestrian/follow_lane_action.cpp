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
  auto parameterToSeeAroundMode = [](std::string_view parameter) {
    if (parameter == "blind") {
      return SeeAroundMode::blind;
    } else if (parameter == "aware") {
      return SeeAroundMode::aware;
    } else {
      THROW_SIMULATION_ERROR("Unknown see_around mode. It must be \"blind\" or \"aware\".");
    }
  };

  should_respect_see_around = parameterToSeeAroundMode(
    common::getParameter<std::string>("pedestrian_ignore_see_around", "blind"));
}

void FollowLaneAction::getBlackBoardValues() { PedestrianActionNode::getBlackBoardValues(); }

bool FollowLaneAction::detectObstacleInLane(
  const lanelet::Ids pedestrian_lanes, const bool see_around) const
{
  if (should_respect_see_around == SeeAroundMode::blind) {
    return false;
  }

  if (!see_around) {
    return false;
  }

  auto hasObstacleInPedestrianLanes =
    [this](const lanelet::Ids pedestrian_lanes_local, const double max_detect_length) {
      using math::geometry::operator-;
      const auto & pedestrian_position = canonicalized_entity_status_->getMapPose().position;
      lanelet::Ids other_entity_lane_ids;
      for (const auto & [_, status] : other_entity_status_) {
        if (status.getType().type != traffic_simulator_msgs::msg::EntityType::EGO) {
          continue;
        }
        if (!status.isInLanelet()) {
          continue;
        }
        const auto norm = math::geometry::norm(status.getMapPose().position - pedestrian_position);
        if (!(norm < max_detect_length)) {
          continue;
        }
        other_entity_lane_ids.push_back(status.getLaneletId());
      }
      std::unordered_set<lanelet::Id> other_lane_id_set(
        other_entity_lane_ids.begin(), other_entity_lane_ids.end());
      for (const auto & pedestrian_lane : pedestrian_lanes_local) {
        if (other_lane_id_set.count(pedestrian_lane)) {
          return true;
        }
      }
      return false;
    };

  auto hasObstacleInFrontOfPedestrian = [this]() {
    using math::geometry::operator-;

    const auto & pedestrian_position = canonicalized_entity_status_->getMapPose().position;

    for (const auto & [_, entity_status] : other_entity_status_) {
      const auto & other_position = entity_status.getMapPose().position;
      const auto relative_position = other_position - pedestrian_position;
      const double relative_angle_rad = std::atan2(relative_position.y, relative_position.x);
      if (relative_angle_rad > 0) {
        return true;
      }
    }
    return false;
  };

  if (hasObstacleInPedestrianLanes(pedestrian_lanes, 10) && hasObstacleInFrontOfPedestrian()) {
    return true;
  } else {
    return false;
  }
}
bool FollowLaneAction::checkPreconditions()
{
  if (
    request_ != traffic_simulator::behavior::Request::NONE &&
    request_ != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return false;
  } else {
    return true;
  }
}

BT::NodeStatus FollowLaneAction::doAction()
{
  if (!canonicalized_entity_status_->isInLanelet()) {
    stopEntity();
    return BT::NodeStatus::RUNNING;
  }
  auto following_lanelets =
    hdmap_utils_->getFollowingLanelets(canonicalized_entity_status_->getLaneletId());
  if (!target_speed_) {
    target_speed_ = hdmap_utils_->getSpeedLimit(following_lanelets);
  }

  const auto obstacle_detector_result =
    detectObstacleInLane(following_lanelets, behavior_parameter_.see_around);
  target_speed_ = obstacle_detector_result ? 0.0 : target_speed_;

  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed_.value()));
  return BT::NodeStatus::RUNNING;
}
}  // namespace pedestrian
}  // namespace entity_behavior
