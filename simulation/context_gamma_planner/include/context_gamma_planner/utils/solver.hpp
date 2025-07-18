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

#ifndef CONTEXT_GAMMA_PLANNER__UTILS_SOLVER_HPP_
#define CONTEXT_GAMMA_PLANNER__UTILS_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <optional>
#include <vector>

#include "context_gamma_planner/utils/math_utils.hpp"
#include "context_gamma_planner/utils/ros_msg_converter.hpp"

namespace context_gamma_planner
{
constexpr double RVO_EPSILON = 0.00001f;

struct line
{
  geometry_msgs::msg::Point point;
  geometry_msgs::msg::Vector3 direction;
};

auto applyConstraintOnLine(
  const std::vector<line> & lines, const line & attention_line, const double limit_speed,
  const geometry_msgs::msg::Vector3 & opt_velocity, const bool direction_opt)
  -> std::optional<geometry_msgs::msg::Vector3>;

auto optimizeVelocityWithConstraints(
  const std::vector<line> & lines, const double limit_speed,
  const geometry_msgs::msg::Vector3 & opt_velocity, const bool direction_opt)
  -> std::optional<geometry_msgs::msg::Vector3>;
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__UTILS_SOLVER_HPP_
