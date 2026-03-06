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

#ifndef CONTEXT_GAMMA_PLANNER__PEDESTRIAN__FOLLOW_LANE_PLANNER_HPP_
#define CONTEXT_GAMMA_PLANNER__PEDESTRIAN__FOLLOW_LANE_PLANNER_HPP_

#include <cmath>
#include <scenario_simulator_exception/exception.hpp>

#include "context_gamma_planner/planner/goal_planner_base.hpp"

namespace context_gamma_planner::pedestrian
{

class FollowLanePlanner : public GoalPlannerBase
{
public:
  explicit FollowLanePlanner(const double goal_threshold);
  ~FollowLanePlanner() final = default;

  void setWaypoints(const std::vector<lanelet::Id> & route_ids);

  auto calculateNextGoalPoint() -> std::optional<geometry_msgs::msg::Point> override;

  void clear() override;
};
}  // namespace context_gamma_planner::pedestrian

#endif  // CONTEXT_GAMMA_PLANNER__PEDESTRIAN__FOLLOW_LANE_PLANNER_HPP_
