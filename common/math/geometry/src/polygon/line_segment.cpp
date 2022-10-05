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

#include <cmath>
#include <geometry/polygon/line_segment.hpp>

namespace math
{
namespace geometry
{
LineSegment::LineSegment(
  const geometry_msgs::msg::Point & start_point, const geometry_msgs::msg::Point & end_point)
: start_point(start_point), end_point(end_point)
{
}

LineSegment::LineSegment(
  const geometry_msgs::msg::Point & start_point, const geometry_msgs::msg::Vector3 & vec,
  double length)
: start_point(start_point), end_point([&]() -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point ret;
    double vec_size = std::hypot(vec.x, vec.y);
    ret.x = start_point.x + vec.x / vec_size * length;
    ret.y = start_point.y + vec.y / vec_size * length;
    ret.z = start_point.z + vec.z / vec_size * length;
    return ret;
  }())
{
}

LineSegment::~LineSegment() {}

bool LineSegment::isIntersect2D(const LineSegment & l0) const
{
  double s, t;
  s = (l0.start_point.x - l0.end_point.x) * (start_point.y - l0.start_point.y) -
      (l0.start_point.y - l0.end_point.y) * (start_point.x - l0.start_point.x);
  t = (l0.start_point.x - l0.end_point.x) * (end_point.y - l0.start_point.y) -
      (l0.start_point.y - l0.end_point.y) * (end_point.x - l0.start_point.x);
  if (s * t > 0) {
    return false;
  }
  s = (start_point.x - end_point.x) * (l0.start_point.y - start_point.y) -
      (start_point.y - end_point.y) * (l0.start_point.x - start_point.x);
  t = (start_point.x - end_point.x) * (l0.end_point.y - start_point.y) -
      (start_point.y - end_point.y) * (l0.end_point.x - start_point.x);
  if (s * t > 0) {
    return false;
  }
  return true;
}

std::optional<geometry_msgs::msg::Point> LineSegment::getIntersection2D(
  const LineSegment & line) const
{
  if (!isIntersect2D(line)) {
    return std::nullopt;
  }
  const auto det = (start_point.x - end_point.x) * (line.end_point.y - line.start_point.y) -
                   (line.end_point.x - line.start_point.x) * (start_point.y - end_point.y);
  const auto t = ((line.end_point.y - line.start_point.y) * (line.end_point.x - end_point.x) +
                  (line.start_point.x - line.end_point.x) * (line.end_point.y - end_point.y)) /
                 det;
  geometry_msgs::msg::Point point;
  point.x = t * start_point.x + (1.0 - t) * end_point.x;
  point.y = t * start_point.y + (1.0 - t) * end_point.y;
  point.z = t * start_point.z + (1.0 - t) * end_point.z;
  return point;
}

geometry_msgs::msg::Vector3 LineSegment::getVector() const
{
  geometry_msgs::msg::Vector3 vec;
  vec.x = end_point.x - start_point.x;
  vec.y = end_point.y - start_point.y;
  vec.z = end_point.z - start_point.z;
  return vec;
}

geometry_msgs::msg::Vector3 LineSegment::get2DVector() const
{
  geometry_msgs::msg::Vector3 vec;
  vec.x = end_point.x - start_point.x;
  vec.y = end_point.y - start_point.y;
  vec.z = 0;
  return vec;
}

double LineSegment::get2DLength() const
{
  return std::hypot(end_point.x - start_point.x, end_point.y - start_point.y);
}

double LineSegment::getLength() const
{
  return std::hypot(
    end_point.x - start_point.x, end_point.y - start_point.y, end_point.z - start_point.z);
}

double LineSegment::getSlope() const
{
  return (end_point.y - start_point.y) / (end_point.x - start_point.x);
}

double LineSegment::getIntercept() const { return end_point.y - getSlope() * end_point.x; }

LineSegment & LineSegment::operator=(const LineSegment &) { return *this; }

std::vector<LineSegment> getLineSegments(const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<LineSegment> seg;
  for (size_t i = 0; i < points.size() - 1; i++) {
    seg.emplace_back(LineSegment(points[i], points[i + 1]));
  }
  seg.emplace_back(LineSegment(points[points.size() - 1], points[0]));
  return seg;
}
}  // namespace geometry
}  // namespace math
