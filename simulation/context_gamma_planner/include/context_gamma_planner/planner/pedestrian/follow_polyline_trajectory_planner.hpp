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

#ifndef CONTEXT_GAMMA_PLANNER__PEDESTRIAN__FOLLOW_POLYLINE_TRAJECTORY_PLANNER_HPP_
#define CONTEXT_GAMMA_PLANNER__PEDESTRIAN__FOLLOW_POLYLINE_TRAJECTORY_PLANNER_HPP_

#include <context_gamma_planner/planner/follow_polyline_trajectory_planner_base.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>

namespace context_gamma_planner
{
namespace pedestrian
{
class FollowPolylineTrajectoryPlanner : public FollowPolylineTrajectoryPlannerBase
{
public:
  FollowPolylineTrajectoryPlanner(const double goal_threshold);
};
}  // namespace pedestrian
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__PEDESTRIAN__FOLLOW_POLYLINE_TRAJECTORY_PLANNER_HPP_
