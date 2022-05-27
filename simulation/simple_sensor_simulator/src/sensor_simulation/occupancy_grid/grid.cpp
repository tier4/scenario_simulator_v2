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
    double vec_size = std::hypot(vec.x, vec.y, vec.z);
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

double LineSegment::getLength() const
{
  return std::hypot(
    end_point.x - start_point.x, end_point.y - start_point.y, end_point.z - start_point.z);
}

std::vector<LineSegment> getLineSegments(const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<LineSegment> seg;
  for (size_t i = 0; i < points.size(); i++) {
    seg.emplace_back(LineSegment(points[i], points[i + 1]));
  }
  return seg;
}

GridCell::GridCell(
  const geometry_msgs::msg::Pose & origin, double size, size_t index, size_t row, size_t col)
: origin(origin), size(size), index(index), row(row), col(col)
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

std::vector<GridCell> Grid::merge(
  const std::vector<GridCell> & cells0, const std::vector<GridCell> & cells1) const
{
  auto ret = cells0;
  std::copy(cells1.begin(), cells1.end(), std::back_inserter(ret));
  std::sort(ret.begin(), ret.end(), [](const GridCell & a, const GridCell & b) {
    return a.index < b.index;
  });
  ret.erase(std::unique(ret.begin(), ret.end()), ret.end());
  return ret;
}

std::vector<size_t> Grid::getFillIndex(const std::vector<GridCell> & cells) const
{
  std::vector<size_t> ret;
  const auto rows = getRows(cells);
  for (const auto & row : rows) {
    auto cell_in_row = filterByRow(cells, row);
    std::sort(cell_in_row.begin(), cell_in_row.end(), [](const GridCell & a, const GridCell & b) {
      return a.col < b.col;
    });
    if (cell_in_row.empty()) {
      continue;
    } else if (cell_in_row.size() == 1) {
      ret.emplace_back(cell_in_row[0].index);
    } else {
      const size_t min_col = cell_in_row[0].col;
      const size_t max_col = cell_in_row[cell_in_row.size() - 1].col;
      for (size_t col = min_col; col <= max_col; col++) {
        ret.emplace_back(width * row + col);
      }
    }
  }
  return ret;
}

Grid::Grid(const geometry_msgs::msg::Pose & origin, double resolution, double height, double width)
: resolution(resolution), height(height), width(width), origin(origin)
{
}

std::vector<GridCell> Grid::getAllCells() const
{
  std::vector<GridCell> ret;
  for (int x_index = 0; x_index <= height; x_index++) {
    for (int y_index = width; y_index <= width; y_index++) {
      geometry_msgs::msg::Pose cell_origin;
      cell_origin.position.x = origin.position.x + (x_index - 0.5 * height) * resolution;
      cell_origin.position.y = origin.position.y + (y_index - 0.5 * width) * resolution;
      cell_origin.position.z = origin.position.z;
      cell_origin.orientation = origin.orientation;
      ret.emplace_back(
        GridCell(cell_origin, resolution, width * y_index + x_index, y_index, x_index));
    }
  }
  return ret;
}

std::vector<GridCell> Grid::getOccupiedCandidates(
  const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive) const
{
  std::vector<GridCell> ret;
  const auto x_max = primitive->getMax(simple_sensor_simulator::Axis::X, origin);
  const auto y_max = primitive->getMax(simple_sensor_simulator::Axis::Y, origin);
  const auto x_min = primitive->getMin(simple_sensor_simulator::Axis::X, origin);
  const auto y_min = primitive->getMin(simple_sensor_simulator::Axis::Y, origin);
  if (!x_max || !y_max || !x_min || !y_min) {
    return ret;
  }
  int x_min_index = std::floor((x_min.get() + resolution * 0.5 * height) / resolution);
  int x_max_index = std::ceil((x_max.get() + resolution * 0.5 * width) / resolution);
  int y_min_index = std::floor((y_min.get() + resolution * 0.5 * height) / resolution);
  int y_max_index = std::ceil((y_max.get() + resolution * 0.5 * width) / resolution);
  for (int x_index = x_min_index; x_index <= x_max_index; x_index++) {
    for (int y_index = y_min_index; y_index <= y_max_index; y_index++) {
      geometry_msgs::msg::Pose cell_origin;
      cell_origin.position.x = origin.position.x + (x_index - 0.5 * height) * resolution;
      cell_origin.position.y = origin.position.y + (y_index - 0.5 * width) * resolution;
      cell_origin.position.z = origin.position.z;
      cell_origin.orientation = origin.orientation;
      ret.emplace_back(
        GridCell(cell_origin, resolution, width * y_index + x_index, y_index, x_index));
    }
  }
  return ret;
}

std::vector<GridCell> Grid::filterByRow(const std::vector<GridCell> & cells, size_t row) const
{
  std::vector<GridCell> ret;
  for (const auto & cell : cells) {
    if (cell.row == row) {
      ret.emplace_back(cell);
    }
  }
  return ret;
}

std::vector<GridCell> Grid::filterByCol(const std::vector<GridCell> & cells, size_t col) const
{
  std::vector<GridCell> ret;
  for (const auto & cell : cells) {
    if (cell.col == col) {
      ret.emplace_back(cell);
    }
  }
  return ret;
}

std::vector<GridCell> Grid::filterByIndex(
  const std::vector<GridCell> & cells, std::vector<size_t> index) const
{
  std::vector<GridCell> ret;
  for (const auto & i : index) {
    const auto find =
      std::find_if(cells.begin(), cells.end(), [&](GridCell cell) { return i == cell.index; });
    if (find != cells.end()) {
      ret.emplace_back(*find);
    }
  }
  return ret;
}

std::vector<GridCell> Grid::filterByIntersection(
  const std::vector<GridCell> & cells, const std::vector<LineSegment> & line_segments) const
{
  std::vector<GridCell> ret;
  for (const auto & cell : cells) {
    if (cell.isIntersect2D(line_segments)) {
      ret.emplace_back(cell);
    }
  }
  return ret;
}

std::vector<GridCell> Grid::filterByContain(
  const std::vector<GridCell> & cells, const std::vector<geometry_msgs::msg::Point> & points) const
{
  std::vector<GridCell> ret;
  for (const auto & cell : cells) {
    if (cell.contains(points)) {
      ret.emplace_back(cell);
    }
  }
  return ret;
}

std::vector<size_t> Grid::getRows(const std::vector<GridCell> & cells) const
{
  std::vector<size_t> ret;
  for (const auto & cell : cells) {
    ret.emplace_back(cell.row);
  }
  std::sort(ret.begin(), ret.end());
  ret.erase(std::unique(ret.begin(), ret.end()), ret.end());
  return ret;
}

std::vector<size_t> Grid::getCols(const std::vector<GridCell> & cells) const
{
  std::vector<size_t> ret;
  for (const auto & cell : cells) {
    ret.emplace_back(cell.col);
  }
  std::sort(ret.begin(), ret.end());
  ret.erase(std::unique(ret.begin(), ret.end()), ret.end());
  return ret;
}

std::vector<GridCell> Grid::getInvisibleCell(
  const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive) const
{
  auto hits = raycastToOutside(primitive);
  std::vector<geometry_msgs::msg::Point> poly = primitive->get2DConvexHull();
  std::copy(poly.begin(), poly.end(), std::back_inserter(hits));
  const auto hull = get2DConvexHull(poly);
  const auto candidates = getAllCells();
  return filterByIndex(
    candidates,
    getFillIndex(merge(
      filterByIntersection(candidates, getLineSegments(hull)), filterByContain(candidates, hull))));
}

std::vector<GridCell> Grid::getOccupiedCell(
  const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive) const
{
  auto candidates = getOccupiedCandidates(primitive);
  const auto points = primitive->get2DConvexHull();
  return filterByIndex(
    candidates, getFillIndex(merge(
                  filterByIntersection(candidates, getLineSegments(points)),
                  filterByContain(candidates, points))));
}

std::array<LineSegment, 4> Grid::getOutsideLineSegments() const
{
  geometry_msgs::msg::Point left_up;
  left_up.x = origin.position.x + resolution * height * 0.5;
  left_up.y = origin.position.y + resolution * height * 0.5;
  left_up.z = origin.position.z;
  geometry_msgs::msg::Point left_down;
  left_down.x = origin.position.x - resolution * height * 0.5;
  left_down.y = origin.position.y + resolution * height * 0.5;
  left_down.z = origin.position.z;
  geometry_msgs::msg::Point right_up;
  right_up.x = origin.position.x + resolution * height * 0.5;
  right_up.y = origin.position.y - resolution * height * 0.5;
  right_up.z = origin.position.z;
  geometry_msgs::msg::Point right_down;
  right_down.x = origin.position.x - resolution * height * 0.5;
  right_down.y = origin.position.y - resolution * height * 0.5;
  right_down.z = origin.position.z;
  return {
    LineSegment(left_up, left_down), LineSegment(left_down, right_down),
    LineSegment(right_down, right_up), LineSegment(right_up, left_up)};
}

std::vector<geometry_msgs::msg::Point> Grid::raycastToOutside(
  const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive) const
{
  std::vector<geometry_msgs::msg::Point> ret;
  const auto outsides = getOutsideLineSegments();
  const auto points = primitive->get2DConvexHull();
  for (const auto & point : points) {
    geometry_msgs::msg::Vector3 vec;
    vec.x = point.x - origin.position.x;
    vec.y = point.y - origin.position.y;
    vec.z = point.z - origin.position.z;
    const auto ray =
      LineSegment(origin.position, vec, std::hypot(width * resolution, height * resolution) * 2);
    for (const auto & outside : outsides) {
      const auto hit = outside.getIntersection2D(ray);
      if (hit) {
        ret.emplace_back(hit.get());
      }
    }
  }
  return ret;
}
}  // namespace simple_sensor_simulator
