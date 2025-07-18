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

#include "context_gamma_planner/utils/orca.hpp"

namespace context_gamma_planner
{

auto ellipse_radius(
  const traffic_simulator_msgs::msg::BoundingBox & bbox, const double relative_angle,
  const double current_angle) -> double
{
  const auto other_major_axis = bbox.dimensions.x * 0.5 * M_SQRT2;
  const auto other_minor_axis = bbox.dimensions.y * 0.5 * M_SQRT2;

  const auto local_phi = relative_angle - current_angle;
  const auto cos_p = std::cos(local_phi);
  const auto sin_p = std::sin(local_phi);

  return (other_major_axis * other_minor_axis) /
         std::sqrt(
           (other_minor_axis * cos_p) * (other_minor_axis * cos_p) +
           (other_major_axis * sin_p) * (other_major_axis * sin_p));
}

auto calculate_orca_line(
  const geometry_msgs::msg::Vector3 & ego_velocity,
  const geometry_msgs::msg::Point & relative_position,
  const geometry_msgs::msg::Vector3 & relative_velocity,
  const traffic_simulator_msgs::msg::BoundingBox & ego_bbox, const double ego_angle,
  const traffic_simulator_msgs::msg::BoundingBox & other_bbox, const double other_angle,
  const double step_time) -> line
{
  const auto inv_time_horizon = 1.0 / 5.0;

  auto cast_to_point = [](const geometry_msgs::msg::Vector3 & p) {
    auto result = geometry_msgs::msg::Point();
    result.x = p.x;
    result.y = p.y;
    result.z = p.z;
    return result;
  };

  const auto dist_sq = sqr(relative_position);

  const auto relative_angle = std::atan2(relative_position.y, relative_position.x);

  const auto ego_radius = ellipse_radius(ego_bbox, relative_angle, ego_angle);
  const auto other_radius = ellipse_radius(other_bbox, M_PI + relative_angle, other_angle);

  const auto combined_radius = ego_radius + other_radius;
  const auto combined_radius_sq = sqr(combined_radius);

  geometry_msgs::msg::Vector3 direction;
  geometry_msgs::msg::Point u;
  if (dist_sq >= combined_radius_sq) {
    const auto w = relative_velocity - relative_position * inv_time_horizon;
    const auto w_length_sq = sqr(w);
    const auto dot_product1 = w.x * relative_position.x + w.y * relative_position.y;
    if (dot_product1 < 0.0f && dot_product1 * dot_product1 > combined_radius_sq * w_length_sq) {
      const auto w_length = std::sqrt(w_length_sq);
      const auto unit_w = w / w_length;
      direction.x = unit_w.y;
      direction.y = unit_w.x * -1.0;
      u = cast_to_point(direction) * (combined_radius * inv_time_horizon - w_length);
    } else {
      const auto leg = std::sqrt(dist_sq - combined_radius_sq);
      if (det(relative_position, w) > 0.0f) {
        direction.x = (relative_position.x * leg - relative_position.y * combined_radius) / dist_sq;
        direction.y = (relative_position.x * combined_radius + relative_position.y * leg) / dist_sq;
      } else {
        direction.x = (relative_position.x * leg + relative_position.y * combined_radius) / dist_sq;
        direction.y =
          (-relative_position.x * combined_radius + relative_position.y * leg) / dist_sq;
        direction = direction * -1.0;
      }
      const auto dot_product2 =
        relative_velocity.x * direction.x + relative_velocity.y * direction.y;
      u = cast_to_point(direction * dot_product2 - relative_velocity);
    }

  } else {
    const auto inv_time_step = 1.0 / step_time;
    const auto w = relative_velocity - relative_position * inv_time_step;
    const auto w_length_sq = sqr(w);
    const auto w_length = std::sqrt(w_length_sq);

    const auto unit_w = w / w_length;
    direction.x = unit_w.y;
    direction.y = unit_w.x * -1.0;
    u = cast_to_point(unit_w * (combined_radius * inv_time_step - w_length));
  }
  const auto point = cast_to_point(ego_velocity + u * 0.5);
  return {point, direction};
}

}  // namespace context_gamma_planner
