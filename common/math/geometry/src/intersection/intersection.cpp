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

#include <geometry/intersection/intersection.hpp>
#include <optional>

namespace math
{
namespace geometry
{
bool isIntersect2D(const LineSegment & line0, const LineSegment & line1)
{
  using Point = geometry_msgs::msg::Point;

  const auto p0 = line0.start_point, q0 = line0.end_point;
  const auto p1 = line1.start_point, q1 = line1.end_point;

  constexpr auto within_bounding_box =
    [](const Point & start, const Point & end, const Point & to_check) -> bool {
    const bool x_inside = ((to_check.x - start.x) * (end.x - to_check.x) >= 0.0);
    const bool y_inside = ((to_check.y - start.y) * (end.y - to_check.y) >= 0.0);
    return x_inside && y_inside;
  };

  constexpr auto orientation =
    [](const Point & start, const Point & end, const Point & to_check) -> double {
    return (end.y - start.y) * (to_check.x - end.x) - (end.x - start.x) * (to_check.y - end.y);
  };

  constexpr auto sign = [](const double x) -> int {
    constexpr double tolerance = 1.0e-10;
    if (x > +tolerance) return +1;
    if (x < -tolerance) return -1;
    return 0;
  };

  const int ori_p0 = sign(orientation(p1, q1, p0));
  const int ori_q0 = sign(orientation(p1, q1, q0));
  const int ori_p1 = sign(orientation(p0, q0, p1));
  const int ori_q1 = sign(orientation(p0, q0, q1));

  // Special case
  // If the lines are collinear; they intersect if and only if their bounding boxes overlap
  if (ori_p1 == 0 && ori_q1 == 0 && ori_p0 == 0 && ori_q0 == 0) {
    return within_bounding_box(p0, q0, p1) || within_bounding_box(p0, q0, p1) ||
           within_bounding_box(p1, q1, p0) || within_bounding_box(p1, q1, p0);
  }

  // General case
  return ori_p1 != ori_q1 && ori_p0 != ori_q0;
}

bool isIntersect2D(const std::vector<LineSegment> & lines)
{
  for (size_t i = 0; i < lines.size(); ++i) {
    for (size_t m = 0; m < lines.size(); ++m) {
      if (i != m && isIntersect2D(lines[i], lines[m])) {
        return true;
      }
    }
  }
  return false;
}

std::optional<geometry_msgs::msg::Point> getIntersection2D(
  const LineSegment & line0, const LineSegment & line1)
{
  if (!isIntersect2D(line0, line1)) {
    return std::nullopt;
  }
  const auto det =
    (line0.start_point.x - line0.end_point.x) * (line1.end_point.y - line1.start_point.y) -
    (line1.end_point.x - line1.start_point.x) * (line0.start_point.y - line0.end_point.y);
  const auto t =
    ((line1.end_point.y - line1.start_point.y) * (line1.end_point.x - line0.end_point.x) +
     (line1.start_point.x - line1.end_point.x) * (line1.end_point.y - line0.end_point.y)) /
    det;
  geometry_msgs::msg::Point point;
  point.x = t * line0.start_point.x + (1.0 - t) * line0.end_point.x;
  point.y = t * line0.start_point.y + (1.0 - t) * line0.end_point.y;
  point.z = t * line0.start_point.z + (1.0 - t) * line0.end_point.z;
  return point;
}

std::vector<geometry_msgs::msg::Point> getIntersection2D(const std::vector<LineSegment> & lines)
{
  std::vector<geometry_msgs::msg::Point> ret;
  for (size_t i = 0; i < lines.size(); ++i) {
    for (size_t m = 0; m < lines.size(); ++m) {
      if (i != m) {
        const auto point = getIntersection2D(lines[i], lines[m]);
        if (point) {
          ret.emplace_back(point.value());
        }
      }
    }
  }
  return ret;
}
}  // namespace geometry
}  // namespace math
