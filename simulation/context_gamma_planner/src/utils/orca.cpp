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

#include "context_gamma_planner/utils/orca.hpp"

namespace context_gamma_planner
{

auto calculate_orca_line(
  const geometry_msgs::msg::Vector3 & ego_velocity,
  const geometry_msgs::msg::Point & relative_position,
  const geometry_msgs::msg::Vector3 & relative_velocity, const Polygon & ego_polygon,
  const Polygon & other_polygon) -> line
{
  const auto inv_time_horizon = 1.0 / 5.0;

  auto calcLineCoefficients = [](const geometry_msgs::msg::Point & position) {
    const double m = (std::abs(position.x) > 1e-4) ? position.y / position.x : 0.0;
    return m;
  };

  auto computeSignedDistanceRange = [](const Polygon & poly, const double a, const double b) {
    auto distanceFromLine = [](const Point & p, const double a, const double b) {
      return (a * p.x() - p.y() + b) / std::sqrt(a * a + 1.0);
    };
    std::vector<std::pair<double, Point>> signed_distance;
    for (auto & pt : poly.outer()) {
      const auto distance = distanceFromLine(pt, a, b);
      signed_distance.emplace_back(distance, pt);
    }
    auto compare = [](const std::pair<double, Point> & a, const std::pair<double, Point> & b) {
      return a.first < b.first;
    };
    const auto [max_dis, max_pt] =
      *std::max_element(signed_distance.begin(), signed_distance.end(), compare);
    return max_dis;
  };

  auto cast_to_point = [](const geometry_msgs::msg::Vector3 & p) {
    auto result = geometry_msgs::msg::Point();
    result.x = p.x;
    result.y = p.y;
    result.z = p.z;
    return result;
  };

  const auto m = calcLineCoefficients(relative_position);

  const auto dist_sq = sqr(relative_position);

  const auto agent_radius = computeSignedDistanceRange(ego_polygon, m, 0.0);
  const auto other_radius = computeSignedDistanceRange(other_polygon, m, 0.0);

  const auto combined_radius = agent_radius + other_radius;
  const auto combined_radius_sq = sqr(combined_radius);

  const auto w = relative_velocity - relative_position * inv_time_horizon;
  const auto w_length_sq = sqr(w);
  const auto dot_product1 = w.x * relative_position.x + w.y * relative_position.y;

  geometry_msgs::msg::Vector3 direction;
  geometry_msgs::msg::Point u;
  if (dot_product1 < 0.0f && dot_product1 * dot_product1 > combined_radius_sq * w_length_sq) {
    const float w_length = std::sqrt(w_length_sq);
    direction = w / w_length;
    u = cast_to_point(direction) * (combined_radius * inv_time_horizon - w_length);
  } else {
    const auto leg = std::sqrt(dist_sq - combined_radius_sq);
    if (det(relative_position, w) > 0.0f) {
      direction.x = (relative_position.x * leg - relative_position.y * combined_radius) / dist_sq;
      direction.y = (relative_position.x * combined_radius + relative_position.y * leg) / dist_sq;
    } else {
      direction.x = (relative_position.x * leg + relative_position.y * combined_radius) / dist_sq;
      direction.y = (-relative_position.x * combined_radius + relative_position.y * leg) / dist_sq;
      direction = direction * -1.0;
    }
    const auto dot_product2 = relative_velocity.x * direction.x + relative_velocity.y * direction.y;
    u = cast_to_point(direction * dot_product2 - relative_velocity);
  }
  const auto point = cast_to_point(ego_velocity + u * 0.5);
  return {point, direction};
}

}  // namespace context_gamma_planner