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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__FOLLOW_TRAJECTORY_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__FOLLOW_TRAJECTORY_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <type_traits>

namespace traffic_simulator
{
namespace follow_trajectory
{
struct Vertex
{
  double time;

  geometry_msgs::msg::Pose position;

  explicit Vertex(const double time, const geometry_msgs::msg::Pose & position)
  : time(time), position(position)
  {
  }
};

struct Polyline
{
  std::vector<Vertex> vertices;
};

template <typename Shape>
struct Parameter  // OpenSCENARIO 1.2 FollowTrajectoryAction
{
  double initial_distance_offset;

  /*
     true if trajectoryFollowingMode == FollowingMode.follow
  */
  bool dynamic_constraints_ignorable;

  /*
     This variable is set to nullopt if the time information defined for the
     waypoint is absolute simulation time. Otherwise, if the time information
     defined for the waypoint is relative time, this variable will be filled
     with the offset from zero simulation time.
  */
  std::optional<double> base_time;

  bool closed;

  Shape shape;

  Parameter() = default;

  explicit Parameter(
    const double initial_distance_offset, const bool dynamic_constraints_ignorable,
    const std::optional<double> base_time, const bool closed, const Shape & shape)
  : initial_distance_offset(initial_distance_offset),
    dynamic_constraints_ignorable(dynamic_constraints_ignorable),
    base_time(base_time),
    closed(closed),
    shape(shape)
  {
  }
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__FOLLOW_TRAJECTORY_HPP_
