// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#ifndef BEHAVIOR_TREE_PLUGIN__ROUTE_PLANNER_HPP_
#define BEHAVIOR_TREE_PLUGIN__ROUTE_PLANNER_HPP_

#include <memory>
#include <queue>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <vector>

namespace traffic_simulator
{
class RoutePlanner
{
public:
  explicit RoutePlanner(std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr);
  std::vector<std::int64_t> getRouteLanelets(
    traffic_simulator_msgs::msg::LaneletPose entity_lanelet_pose,
    std::vector<traffic_simulator_msgs::msg::LaneletPose> waypoints, double horizon = 100);
  std::vector<std::int64_t> getRouteLanelets(
    traffic_simulator_msgs::msg::LaneletPose entity_lanelet_pose,
    traffic_simulator_msgs::msg::LaneletPose target_lanelet_pose, double horizon = 100);
  std::vector<std::int64_t> getRouteLanelets(
    traffic_simulator_msgs::msg::LaneletPose entity_lanelet_pose, double horizon = 100);
  void cancelGoal();
  std::vector<traffic_simulator_msgs::msg::LaneletPose> getGoalPoses();
  std::vector<geometry_msgs::msg::Pose> getGoalPosesInWorldFrame();

private:
  void cancelGoal(const traffic_simulator_msgs::msg::LaneletPose & entity_lanelet_pose);
  void plan(
    traffic_simulator_msgs::msg::LaneletPose entity_lanelet_pose,
    traffic_simulator_msgs::msg::LaneletPose target_lanelet_pose);
  boost::optional<std::vector<std::int64_t>> whole_route_;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  std::queue<traffic_simulator_msgs::msg::LaneletPose> waypoint_queue_;
};
}  // namespace traffic_simulator

#endif  // BEHAVIOR_TREE_PLUGIN__ROUTE_PLANNER_HPP_
