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

#include "context_gamma_planner/utils/solver.hpp"

namespace context_gamma_planner
{
auto applyConstraintOnLine(
  const std::vector<line> & lines, const line & attention_line, const double limit_speed,
  const geometry_msgs::msg::Vector3 & opt_velocity, const bool direction_opt)
  -> std::optional<geometry_msgs::msg::Vector3>
{
  const auto & [p, d] = attention_line;
  const auto dot_product = p * d;
  const auto discriminant = sqr(dot_product) + sqr(limit_speed) - sqr(p);

  if (discriminant < 0.0) {
    return std::nullopt;
  }

  const auto sqrt_discriminant = std::sqrt(discriminant);
  auto t_left = -dot_product - sqrt_discriminant;
  auto t_right = -dot_product + sqrt_discriminant;

  for (const auto & [pi, di] : lines) {
    const auto denominator = det(d, di);
    const auto numerator = det(di, p - pi);

    if (std::fabs(denominator) <= RVO_EPSILON) {
      if (numerator < 0.0f) {
        return std::nullopt;
      } else {
        continue;
      }
    }

    const auto t = numerator / denominator;

    if (denominator >= 0.0f) {
      t_right = std::min(t_right, t);
    } else {
      t_left = std::max(t_left, t);
    }
    if (t_left > t_right) {
      return std::nullopt;
    }
  }

  if (direction_opt) {
    if (opt_velocity * d > 0.0) {
      return cast_to_vec(p) + d * t_right;
    } else {
      return cast_to_vec(p) + d * t_left;
    }
  } else {
    const auto t = d * (opt_velocity - p);

    if (t < t_left) {
      return cast_to_vec(p) + d * t_left;
    } else if (t > t_right) {
      return cast_to_vec(p) + d * t_right;
    } else {
      return cast_to_vec(p) + d * t;
    }
  }
}

auto optimizeVelocityWithConstraints(
  const std::vector<line> & lines, const double limit_speed,
  const geometry_msgs::msg::Vector3 & opt_velocity, const bool direction_opt)
  -> std::optional<geometry_msgs::msg::Vector3>
{
  auto velocity = geometry_msgs::msg::Vector3();
  if (direction_opt) {
    velocity = opt_velocity * limit_speed;
  } else if (sqr(opt_velocity) > sqr(limit_speed)) {
    velocity = normalize(opt_velocity) * limit_speed;
  } else {
    velocity = opt_velocity;
  }
  for (const auto & line : lines) {
    if (det(line.direction, line.point - velocity) > 0.0) {
      const auto result =
        applyConstraintOnLine(lines, line, limit_speed, opt_velocity, direction_opt);
      if (result) {
        velocity = result.value();
      }
    }
  }
  return velocity;
}
}  // namespace context_gamma_planner
