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
namespace
{
using math::geometry::operator+;
using math::geometry::operator-;
using math::geometry::operator*;

inline double det(const geometry_msgs::msg::Vector3 & a, const geometry_msgs::msg::Vector3 & b)
{
  return math::geometry::cross2d(a, b);
}
}  // namespace

auto applyConstraintOnLine(
  const std::vector<line> & constraint_lines, std::size_t line_no, const double maximum_speed,
  const geometry_msgs::msg::Vector3 & preferred_velocity, const bool prioritize_direction_alignment)
  -> std::optional<geometry_msgs::msg::Vector3>
{
  using math::geometry::operator+;
  using math::geometry::operator-;
  using math::geometry::operator*;

  const auto & constraint_point = constraint_lines[line_no].point;
  const auto & constraint_direction = constraint_lines[line_no].direction;

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

  // Only consider previously satisfied constraints [0, line_no)
  for (std::size_t i = 0; i < line_no; ++i) {
    const auto & other_constraint_point = constraint_lines[i].point;
    const auto & other_constraint_direction = constraint_lines[i].direction;
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

// Solve the 2D linear program over the set of lines. Returns the index of the first
// constraint that failed, or lines.size() if all constraints are satisfied.
static std::size_t linearProgram2(
  const std::vector<line> & lines, double radius, const geometry_msgs::msg::Vector3 & opt_velocity,
  bool direction_opt, geometry_msgs::msg::Vector3 & result)
{
  if (direction_opt) {
    result = opt_velocity * radius;
  } else if (math::geometry::innerProduct(opt_velocity, opt_velocity) > radius * radius) {
    result = math::geometry::normalize(opt_velocity) * radius;
  } else {
    result = opt_velocity;
  }

  for (std::size_t i = 0; i < lines.size(); ++i) {
    if (math::geometry::cross2d(lines[i].direction, lines[i].point - result) > 0.0) {
      const auto temp_result = result;
      const auto new_result = applyConstraintOnLine(lines, i, radius, opt_velocity, direction_opt);
      if (!new_result) {
        result = temp_result;
        return i;
      }
      result = new_result.value();
    }
  }

  return lines.size();
}

// Handle failure at `begin_line` by searching along intersections of constraint boundaries.
static void linearProgram3(
  const std::vector<line> & lines, std::size_t begin_line, double radius,
  geometry_msgs::msg::Vector3 & result)
{
  double distance = 0.0;
  std::vector<line> proj_lines;

  for (std::size_t i = begin_line; i < lines.size(); ++i) {
    const auto violation = math::geometry::cross2d(lines[i].direction, lines[i].point - result);
    if (violation > distance) {
      proj_lines.clear();

      for (std::size_t j = 0; j < i; ++j) {
        line l;
        const double determinant = math::geometry::cross2d(lines[i].direction, lines[j].direction);

        if (std::fabs(determinant) <= ORCA_EPSILON) {
          if (math::geometry::innerProduct(lines[i].direction, lines[j].direction) > 0.0) {
            // Line i and line j point in the same direction.
            continue;
          } else {
            // Line i and line j point in opposite direction.
            l.point = (lines[i].point + lines[j].point) * 0.5;
          }
        } else {
          l.point = lines[i].point +
                    lines[i].direction * (math::geometry::cross2d(
                                            lines[j].direction, lines[i].point - lines[j].point) /
                                          determinant);
        }

        l.direction = math::geometry::normalize(lines[j].direction - lines[i].direction);
        proj_lines.push_back(l);
      }

      const auto temp_result = result;
      geometry_msgs::msg::Vector3 dir_opt;
      dir_opt.x = -lines[i].direction.y;
      dir_opt.y = lines[i].direction.x;
      dir_opt.z = 0.0;

      if (linearProgram2(proj_lines, radius, dir_opt, true, result) < proj_lines.size()) {
        // This should in principle not happen; keep previous result.
        result = temp_result;
      }

      distance = math::geometry::cross2d(lines[i].direction, lines[i].point - result);
    }
  }
}

auto optimizeVelocityWithConstraints(
  const std::vector<line> & constraint_lines, const double maximum_speed,
  const geometry_msgs::msg::Vector3 & preferred_velocity, const bool prioritize_direction_alignment)
  -> std::optional<geometry_msgs::msg::Vector3>
{
  geometry_msgs::msg::Vector3 result;
  const auto line_fail = linearProgram2(
    constraint_lines, maximum_speed, preferred_velocity, prioritize_direction_alignment, result);

  if (line_fail < constraint_lines.size()) {
    // We have no obstacle lines in this usage; start from the failing line.
    linearProgram3(constraint_lines, line_fail, maximum_speed, result);
  }

  return result;
}
}  // namespace context_gamma_planner
