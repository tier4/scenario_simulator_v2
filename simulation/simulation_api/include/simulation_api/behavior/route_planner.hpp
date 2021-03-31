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

#ifndef SIMULATION_API__BEHAVIOR__ROUTE_PLANNER_HPP_
#define SIMULATION_API__BEHAVIOR__ROUTE_PLANNER_HPP_

#include <simulation_api/hdmap_utils/hdmap_utils.hpp>

#include <memory>
#include <vector>

namespace simulation_api
{
class RoutePlanner
{
public:
  explicit RoutePlanner(std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr);
  std::vector<std::int64_t> getRouteLanelets(
    openscenario_msgs::msg::LaneletPose entity_lanelet_pose,
    std::vector<openscenario_msgs::msg::LaneletPose> waypoints,
    double horizon = 100);
  std::vector<std::int64_t> getRouteLanelets(
    openscenario_msgs::msg::LaneletPose entity_lanelet_pose,
    openscenario_msgs::msg::LaneletPose target_lanelet_pose,
    double horizon = 100);
  std::vector<std::int64_t> getRouteLanelets(
    openscenario_msgs::msg::LaneletPose entity_lanelet_pose,
    double horizon = 100);
  void cancelGoal();

private:
  void cancelGoal(const openscenario_msgs::msg::LaneletPose & entity_lanelet_pose);
  void plan(
    openscenario_msgs::msg::LaneletPose entity_lanelet_pose,
    openscenario_msgs::msg::LaneletPose target_lanelet_pose);
  boost::optional<std::vector<std::int64_t>> whole_route_;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  std::queue<openscenario_msgs::msg::LaneletPose> waypoint_queue_;
};
}  // namespace simulation_api

#endif  // SIMULATION_API__BEHAVIOR__ROUTE_PLANNER_HPP_
