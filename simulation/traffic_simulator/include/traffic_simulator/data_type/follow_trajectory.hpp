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
     This data member corresponds to the attribute
     TimeReference.Timing.domainAbsoluteRelative of ASAM OpenSCENARIO 1.2. the
     OpenSCENARIO standard does not define the behavior when the value of
     domainAbsoluteRelative is "relative". The standard only states "Definition
     of time value context as either absolute or relative", and it is
     completely unclear when the relative time starts.

     This implementation has interpreted the specification as follows: Relative
     time starts from the start of FollowTrajectoryAction or from the time of
     reaching the previous "waypoint with arrival time".

     Note: If Parameter::closed is true, this variable is always assigned
     false.
  */
  bool time_specification_of_trajectory_is_absolute_simulation_time;

  bool closed;

  Shape shape;

  Parameter() = default;

  explicit Parameter(
    const double initial_distance_offset, const bool dynamic_constraints_ignorable,
    const bool time_specification_of_trajectory_is_absolute_simulation_time, const bool closed,
    const Shape & shape)
  : initial_distance_offset(initial_distance_offset),
    dynamic_constraints_ignorable(dynamic_constraints_ignorable),
    time_specification_of_trajectory_is_absolute_simulation_time(
      time_specification_of_trajectory_is_absolute_simulation_time and not closed),
    closed(closed),
    shape(shape)
  {
  }
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__FOLLOW_TRAJECTORY_HPP_
