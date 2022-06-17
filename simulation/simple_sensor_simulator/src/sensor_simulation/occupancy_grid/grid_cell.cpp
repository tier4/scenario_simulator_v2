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
GridCell::GridCell(
  const geometry_msgs::msg::Pose & origin, double size, size_t index, size_t row, size_t col)
: origin(origin), size(size), index(index), row(row), col(col), data_(0)
{
}

std::array<geometry_math::LineSegment, 4> GridCell::getLineSegments() const
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
  std::array<geometry_math::LineSegment, 4> ret = {
    geometry_math::LineSegment(left_up, left_down),
    geometry_math::LineSegment(left_down, right_down),
    geometry_math::LineSegment(right_down, right_up),
    geometry_math::LineSegment(right_up, left_up)};
  return ret;
}

std::vector<geometry_msgs::msg::Point> getIntersection2D(
  const std::vector<geometry_math::LineSegment> & lines)
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

bool GridCell::isIntersect2D(const geometry_math::LineSegment & line) const
{
  for (const auto & outside : getLineSegments()) {
    if (outside.isIntersect2D(line)) {
      return true;
    }
  }
  return false;
}

bool GridCell::isIntersect2D(const std::vector<geometry_math::LineSegment> & line_segments) const
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
}  // namespace simple_sensor_simulator
