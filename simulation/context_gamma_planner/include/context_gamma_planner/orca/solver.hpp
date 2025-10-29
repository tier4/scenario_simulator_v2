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

#ifndef CONTEXT_GAMMA_PLANNER__ORCA_SOLVER_HPP_
#define CONTEXT_GAMMA_PLANNER__ORCA_SOLVER_HPP_

#include <geometry/vector3/cross_2d.hpp>
#include <geometry/vector3/inner_product.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry/vector3/ros_msg_converter.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <optional>
#include <vector>

namespace context_gamma_planner
{
constexpr double ORCA_EPSILON = 0.00001f;

struct line
{
  geometry_msgs::msg::Point point;
  geometry_msgs::msg::Vector3 direction;
};

// Solve a 1D linear program on the boundary of `constraint_lines[line_no]` under
// the previously satisfied constraints [0, line_no).
auto applyConstraintOnLine(
  const std::vector<line> & constraint_lines, std::size_t line_no, const double maximum_speed,
  const geometry_msgs::msg::Vector3 & preferred_velocity, const bool prioritize_direction_alignment)
  -> std::optional<geometry_msgs::msg::Vector3>;

auto optimizeVelocityWithConstraints(
  const std::vector<line> & constraint_lines, const double maximum_speed,
  const geometry_msgs::msg::Vector3 & preferred_velocity, const bool prioritize_direction_alignment)
  -> std::optional<geometry_msgs::msg::Vector3>;
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__ORCA_SOLVER_HPP_
