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

#include "context_gamma_planner/utils/collider.hpp"

namespace context_gamma_planner
{
auto create_box_polygon(double width, double length) -> Polygon
{
  Polygon poly;
  poly.outer().emplace_back(Point(0.0, 0.0));
  poly.outer().emplace_back(Point(length, 0.0));
  poly.outer().emplace_back(Point(length, width));
  poly.outer().emplace_back(Point(0.0, width));
  poly.outer().emplace_back(Point(0.0, 0.0));
  return poly;
}

auto polygon_from_bbox(const traffic_simulator_msgs::msg::BoundingBox & bbox) -> Polygon
{
  const auto length = bbox.dimensions.x;
  const auto width = bbox.dimensions.y;
  Polygon poly = create_box_polygon(width, length);
  for (auto & p : poly.outer()) {
    p.x(p.x() + bbox.center.x);
    p.y(p.y() + bbox.center.y);
  }
  return poly;
}

auto rotate_polygon(const Polygon & poly, const geometry_msgs::msg::Quaternion & orientation)
  -> Polygon
{
  Polygon rotated_poly;
  const auto angle = math::geometry::convertQuaternionToEulerAngle(orientation).z;
  for (const auto & point : poly.outer()) {
    double x_rotated = point.x() * std::cos(angle) - point.y() * std::sin(angle);
    double y_rotated = point.x() * std::sin(angle) + point.y() * std::cos(angle);
    rotated_poly.outer().emplace_back(Point(x_rotated, y_rotated));
  }
  return rotated_poly;
}
}  // namespace context_gamma_planner