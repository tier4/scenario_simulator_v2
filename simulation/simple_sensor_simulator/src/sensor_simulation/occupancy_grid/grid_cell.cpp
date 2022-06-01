// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <quaternion_operation/quaternion_operation.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid.hpp>

namespace simple_sensor_simulator
{
std::vector<geometry_msgs::msg::Point> get2DConvexHull(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  typedef boost::geometry::model::d2::point_xy<double> boost_point;
  typedef boost::geometry::model::polygon<boost_point> boost_polygon;
  boost_polygon poly;
  for (const auto & p : points) {
    boost::geometry::exterior_ring(poly).push_back(boost_point(p.x, p.y));
  }
  boost_polygon hull;
  boost::geometry::convex_hull(poly, hull);
  std::vector<geometry_msgs::msg::Point> polygon;
  for (auto it = boost::begin(boost::geometry::exterior_ring(hull));
       it != boost::end(boost::geometry::exterior_ring(hull)); ++it) {
    double x = boost::geometry::get<0>(*it);
    double y = boost::geometry::get<1>(*it);
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    polygon.emplace_back(p);
  }
  return polygon;
}

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

boost::optional<geometry_msgs::msg::Point> LineSegment::getIntersection2D(
  const LineSegment & line) const
{
  if (!isIntersect2D(line)) {
    return boost::none;
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

std::vector<LineSegment> getLineSegments(const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<LineSegment> seg;
  for (size_t i = 0; i < points.size() - 1; i++) {
    seg.emplace_back(LineSegment(points[i], points[i + 1]));
  }
  seg.emplace_back(LineSegment(points[points.size() - 1], points[0]));
  return seg;
}

LineSegment & LineSegment::operator=(const LineSegment &) { return *this; }

GridCell::GridCell(
  const geometry_msgs::msg::Pose & origin, double size, size_t index, size_t row, size_t col)
: origin(origin), size(size), index(index), row(row), col(col), data_(0)
{
}

std::array<LineSegment, 4> GridCell::getLineSegments() const
{
  geometry_msgs::msg::Point left_up;
  left_up.x = size * 0.5;
  left_up.y = size * 0.5;
  left_up = transformToWorld(left_up);
  geometry_msgs::msg::Point left_down;
  left_down.x = size * 0.5;
  left_down.y = -size * 0.5;
  left_down = transformToWorld(left_down);
  geometry_msgs::msg::Point right_up;
  right_up.x = -size * 0.5;
  right_up.y = size * 0.5;
  right_up = transformToWorld(right_up);
  geometry_msgs::msg::Point right_down;
  right_down.x = -size * 0.5;
  right_down.y = -size * 0.5;
  right_down = transformToWorld(right_down);
  std::array<LineSegment, 4> ret = {
    LineSegment(left_up, left_down), LineSegment(left_down, right_down),
    LineSegment(right_down, right_up), LineSegment(right_up, left_up)};
  return ret;
}

std::vector<geometry_msgs::msg::Point> getIntersection2D(const std::vector<LineSegment> & lines)
{
  std::vector<geometry_msgs::msg::Point> ret;
  for (size_t i = 0; i < lines.size(); i++) {
    for (size_t m = 0; m < lines.size(); m++) {
      if (i < m) {
        const auto intersection = lines[i].getIntersection2D(lines[m]);
        if (intersection) {
          ret.emplace_back(intersection.get());
        }
      }
    }
  }
  return ret;
}

geometry_msgs::msg::Point GridCell::transformToWorld(const geometry_msgs::msg::Point & point) const
{
  auto mat = quaternion_operation::getRotationMatrix(origin.orientation);
  Eigen::VectorXd p(3);
  p(0) = point.x;
  p(1) = point.y;
  p(2) = point.z;
  p = mat * p;
  p(0) = p(0) + origin.position.x;
  p(1) = p(1) + origin.position.y;
  p(2) = p(2) + origin.position.z;
  geometry_msgs::msg::Point ret;
  ret.x = p(0);
  ret.y = p(1);
  ret.z = p(2);
  return ret;
}

bool GridCell::isIntersect2D(const LineSegment & line) const
{
  for (const auto & outside : getLineSegments()) {
    if (outside.isIntersect2D(line)) {
      return true;
    }
  }
  return false;
}

bool GridCell::isIntersect2D(const std::vector<LineSegment> & line_segments) const
{
  for (const auto & line_segment : line_segments) {
    if (isIntersect2D(line_segment)) {
      return true;
    }
  }
  return false;
}

bool GridCell::contains(const geometry_msgs::msg::Point & p) const
{
  const auto line_segments = getLineSegments();
  typedef boost::geometry::model::d2::point_xy<double> boost_point;
  typedef boost::geometry::model::polygon<boost_point> boost_polygon;
  boost_polygon poly;
  for (const auto & line_segment : line_segments) {
    boost::geometry::exterior_ring(poly).push_back(
      boost_point(line_segment.start_point.x, line_segment.start_point.y));
  }
  return boost::geometry::within(boost_point(p.x, p.y), poly);
}

bool GridCell::contains(const std::vector<geometry_msgs::msg::Point> & points) const
{
  for (const auto & point : points) {
    if (contains(point)) {
      return true;
    }
  }
  return false;
}

int8_t GridCell::getData() const { return data_; }
void GridCell::setData(int8_t data) { data_ = data; }

bool GridCell::operator==(const GridCell & rhs) const
{
  if (rhs.row == row && rhs.col == col && rhs.index == index) {
    return true;
  }
  return false;
}

bool GridCell::operator!=(const GridCell & rhs) const { return !(*this == rhs); }

bool GridCell::operator<(const GridCell & rhs) const { return index < rhs.index; }
bool GridCell::operator>(const GridCell & rhs) const { return rhs < *this; }
bool GridCell::operator<=(const GridCell & rhs) const { return !(rhs > *this); }
bool GridCell::operator>=(const GridCell & rhs) const { return !(rhs < *this); }
GridCell & GridCell::operator=(const GridCell &) { return *this; }
}  // namespace simple_sensor_simulator
