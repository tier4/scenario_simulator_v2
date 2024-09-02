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
#include <limits>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>

namespace math
{
namespace geometry
{
bool isIntersect2D(const LineSegment & line0, const LineSegment & line1)
{
  const auto &p0 = line0.start_point, &q0 = line0.end_point;
  const auto &p1 = line1.start_point, &q1 = line1.end_point;

  const int relative_position_p0 = line1.relativePointPosition2D(p0);
  const int relative_position_q0 = line1.relativePointPosition2D(q0);
  const int relative_position_p1 = line0.relativePointPosition2D(p1);
  const int relative_position_q1 = line0.relativePointPosition2D(q1);

  if (
    relative_position_p1 == 0 && relative_position_q1 == 0 && relative_position_p0 == 0 &&
    relative_position_q0 == 0) {
    return line0.isInBounds2D(p1) || line0.isInBounds2D(q1) || line1.isInBounds2D(p0) ||
           line1.isInBounds2D(q0);
  } else {
    return relative_position_p1 != relative_position_q1 &&
           relative_position_p0 != relative_position_q0;
  }
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
  if (not line0.isIntersect2D(line1)) {
    return std::nullopt;
  } else {
    // 'line0' represented as a0x + b0y = c0
    const double a0 = line0.vector_2d.y;
    const double b0 = -line0.vector_2d.x;
    const double c0 = a0 * line0.start_point.x + b0 * line0.start_point.y;

    // 'line1' represented as a1x + b1y = c1
    const double a1 = line1.vector_2d.y;
    const double b1 = -line1.vector_2d.x;
    const double c1 = a1 * line1.start_point.x + b1 * line1.start_point.y;

    const double determinant = a0 * b1 - a1 * b0;

    if (std::abs(determinant) <= std::numeric_limits<double>::epsilon()) {
      // The lines do intersect, but they are collinear and overlap.
      THROW_SIMULATION_ERROR(
        "Line segments are collinear. So determinant is zero.",
        "If this message was displayed, something completely unexpected happens.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    } else {
      return geometry_msgs::build<geometry_msgs::msg::Point>()
        .x((b1 * c0 - b0 * c1) / determinant)
        .y((a0 * c1 - a1 * c0) / determinant)
        .z(0.0);
    }
  }
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
