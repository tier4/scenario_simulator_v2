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
  double s, t;
  s = (line0.start_point.x - line0.end_point.x) * (line1.start_point.y - line0.start_point.y) -
      (line0.start_point.y - line0.end_point.y) * (line1.start_point.x - line0.start_point.x);
  t = (line0.start_point.x - line0.end_point.x) * (line1.end_point.y - line0.start_point.y) -
      (line0.start_point.y - line0.end_point.y) * (line1.end_point.x - line0.start_point.x);
  if (s * t > 0) {
    return false;
  }
  s = (line1.start_point.x - line1.end_point.x) * (line0.start_point.y - line1.start_point.y) -
      (line1.start_point.y - line1.end_point.y) * (line0.start_point.x - line1.start_point.x);
  t = (line1.start_point.x - line1.end_point.x) * (line0.end_point.y - line1.start_point.y) -
      (line1.start_point.y - line1.end_point.y) * (line0.end_point.x - line1.start_point.x);
  if (s * t > 0) {
    return false;
  }
  return true;
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
