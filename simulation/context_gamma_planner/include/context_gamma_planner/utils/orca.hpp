// Copyright 2021 Tier IV, Inc All rights reserved.
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

#ifndef CONTEXT_GAMMA_PLANNER__UTILS_ORCA_HPP_
#define CONTEXT_GAMMA_PLANNER__UTILS_ORCA_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <optional>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <vector>

#include "context_gamma_planner/utils/math_utils.hpp"
#include "context_gamma_planner/utils/solver.hpp"

namespace context_gamma_planner
{
auto ellipse_radius(
  const traffic_simulator_msgs::msg::BoundingBox & bbox, const double relative_angle,
  const double current_angle) -> double;

auto calculate_orca_line(
  const geometry_msgs::msg::Vector3 & ego_velocity,
  const geometry_msgs::msg::Point & relative_position,
  const geometry_msgs::msg::Vector3 & relative_velocity,
  const traffic_simulator_msgs::msg::BoundingBox & ego_bbox, const double ego_angle,
  const traffic_simulator_msgs::msg::BoundingBox & other_bbox, const double other_angle,
  const double step_time) -> line;
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__UTILS_ORCA_HPP_
