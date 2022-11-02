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

#include <scenario_simulator_exception/exception.hpp>
#include <type_traits>

namespace traffic_simulator
{
namespace follow_trajectory
{
struct Vertex
{
  /*
     absolute SimulationTime. The OpenSCENARIO interpreter is responsible for
     setting the proper time.
  */
  std::optional<double> time;

  geometry_msgs::msg::Pose position;

  explicit Vertex(const std::optional<double> & time, const geometry_msgs::msg::Pose & position)
  : time(time), position(position)
  {
  }
};

struct Polyline
{
  std::vector<Vertex> vertices;
};

struct Clothoid
{
  // TODO
};

struct Nurbs
{
  // TODO
};

template <typename Shape>
struct Parameter
{
  double initial_distance_offset;

  bool dynamic_constraints_ignorable;

  bool closed;

  Shape shape;

  explicit Parameter(
    const double initial_distance_offset, const bool dynamic_constraints_ignorable,
    const bool closed, const Shape & shape)
  : initial_distance_offset(initial_distance_offset),
    dynamic_constraints_ignorable(dynamic_constraints_ignorable),
    closed(closed),
    shape(shape)
  {
  }
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__FOLLOW_TRAJECTORY_HPP_
