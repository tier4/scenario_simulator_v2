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

#include "context_gamma_planner/orca/solver.hpp"

namespace context_gamma_planner
{
auto applyConstraintOnLine(
  const std::vector<line> & constraint_lines, const line & target_constraint_line,
  const double maximum_speed, const geometry_msgs::msg::Vector3 & preferred_velocity,
  const bool prioritize_direction_alignment) -> std::optional<geometry_msgs::msg::Vector3>
{
  using math::geometry::operator+;

  using math::geometry::operator-;
  using math::geometry::operator*;

  const auto & [constraint_point, constraint_direction] = target_constraint_line;
  const auto point_direction_projection =
    math::geometry::innerProduct(constraint_point, constraint_direction);
  const auto discriminant = point_direction_projection * point_direction_projection +
                            maximum_speed * maximum_speed -
                            math::geometry::innerProduct(constraint_point, constraint_point);

  if (discriminant < 0.0) {
    return std::nullopt;
  }

  const auto sqrt_discriminant = std::sqrt(discriminant);
  auto parameter_lower_bound = -point_direction_projection - sqrt_discriminant;
  auto parameter_upper_bound = -point_direction_projection + sqrt_discriminant;

  // NOTE: RVO2 keeps agent positions in a k-d tree and achieves roughly O(N log N)
  // neighbor queries, while this approach checks every pair directly, so it remains
  // a simpler but O(N^2) brute-force pass for now.
  for (const auto & constraint_line : constraint_lines) {
    const auto & other_constraint_point = constraint_line.point;
    const auto & other_constraint_direction = constraint_line.direction;
    const auto denominator =
      math::geometry::cross2d(constraint_direction, other_constraint_direction);
    const auto numerator = math::geometry::cross2d(
      other_constraint_direction, constraint_point - other_constraint_point);

    if (std::fabs(denominator) <= ORCA_EPSILON) {
      if (numerator < 0.0f) {
        return std::nullopt;
      } else {
        continue;
      }
    }

    const auto intersection_parameter = numerator / denominator;

    if (denominator >= 0.0f) {
      parameter_upper_bound = std::min(parameter_upper_bound, intersection_parameter);
    } else {
      parameter_lower_bound = std::max(parameter_lower_bound, intersection_parameter);
    }
    if (parameter_lower_bound > parameter_upper_bound) {
      return std::nullopt;
    }
  }

  if (prioritize_direction_alignment) {
    if (math::geometry::innerProduct(preferred_velocity, constraint_direction) > 0.0) {
      return math::geometry::castToVec(constraint_point) +
             constraint_direction * parameter_upper_bound;
    } else {
      return math::geometry::castToVec(constraint_point) +
             constraint_direction * parameter_lower_bound;
    }
  } else {
    const auto preferred_parameter =
      math::geometry::innerProduct(constraint_direction, (preferred_velocity - constraint_point));

    if (preferred_parameter < parameter_lower_bound) {
      return math::geometry::castToVec(constraint_point) +
             constraint_direction * parameter_lower_bound;
    } else if (preferred_parameter > parameter_upper_bound) {
      return math::geometry::castToVec(constraint_point) +
             constraint_direction * parameter_upper_bound;
    } else {
      return math::geometry::castToVec(constraint_point) +
             constraint_direction * preferred_parameter;
    }
  }
}

auto optimizeVelocityWithConstraints(
  const std::vector<line> & constraint_lines, const double maximum_speed,
  const geometry_msgs::msg::Vector3 & preferred_velocity, const bool prioritize_direction_alignment)
  -> std::optional<geometry_msgs::msg::Vector3>
{
  using math::geometry::operator-;
  using math::geometry::operator*;

  auto optimized_velocity = geometry_msgs::msg::Vector3();
  if (prioritize_direction_alignment) {
    optimized_velocity = preferred_velocity * maximum_speed;
  } else if (
    math::geometry::innerProduct(preferred_velocity, preferred_velocity) >
    maximum_speed * maximum_speed) {
    optimized_velocity = math::geometry::normalize(preferred_velocity) * maximum_speed;
  } else {
    optimized_velocity = preferred_velocity;
  }

  for (const auto & constraint_line : constraint_lines) {
    if (
      math::geometry::cross2d(
        constraint_line.direction, constraint_line.point - optimized_velocity) > 0.0) {
      const auto adjusted_velocity = applyConstraintOnLine(
        constraint_lines, constraint_line, maximum_speed, preferred_velocity,
        prioritize_direction_alignment);
      if (adjusted_velocity) {
        optimized_velocity = adjusted_velocity.value();
      }
    }
  }
  return optimized_velocity;
}
}  // namespace context_gamma_planner
