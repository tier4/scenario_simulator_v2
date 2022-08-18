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

#include <quaternion_operation/quaternion_operation.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid.hpp>

namespace simple_sensor_simulator
{
Grid::Grid(
  double resolution, size_t height, size_t width, int8_t occupied_cost, int8_t invisible_cost)
: resolution(resolution),
  height(height),
  width(width),
  occupied_cost(occupied_cost),
  invisible_cost(invisible_cost),
  values_(height * width)
{ }

double Grid::getDiagonalLength() const { return std::hypot(width, height) * resolution; }

geometry_msgs::msg::Point Grid::transformToGrid(const geometry_msgs::msg::Point & world_point) const
{
  auto mat =
    quaternion_operation::getRotationMatrix(quaternion_operation::conjugate(origin_.orientation));
  Eigen::VectorXd p(3);
  p(0) = world_point.x;
  p(1) = world_point.y;
  p(2) = world_point.z;
  p = mat * p;
  p(0) = p(0) - origin_.position.x;
  p(1) = p(1) - origin_.position.y;
  p(2) = p(2) - origin_.position.z;
  geometry_msgs::msg::Point ret;
  ret.x = p(0);
  ret.y = p(1);
  ret.z = p(2);
  return ret;
}

math::geometry::LineSegment Grid::transformToGrid(const math::geometry::LineSegment & line) const
{
  return math::geometry::LineSegment(
    transformToGrid(line.start_point), transformToGrid(line.end_point));
}

geometry_msgs::msg::Point Grid::transformToWorld(const geometry_msgs::msg::Point & grid_point) const
{
  auto mat = quaternion_operation::getRotationMatrix(origin_.orientation);
  Eigen::VectorXd p(3);
  p(0) = grid_point.x;
  p(1) = grid_point.y;
  p(2) = grid_point.z;
  p = mat * p;
  p(0) = p(0) + origin_.position.x;
  p(1) = p(1) + origin_.position.y;
  p(2) = p(2) + origin_.position.z;
  geometry_msgs::msg::Point ret;
  ret.x = p(0);
  ret.y = p(1);
  ret.z = p(2);
  return ret;
}

geometry_msgs::msg::Point Grid::transformToPixel(const geometry_msgs::msg::Point & grid_point) const
{
  geometry_msgs::msg::Point p;
  p.x = (grid_point.x + height * resolution * 0.5) / resolution;
  p.y = (grid_point.y + width * resolution * 0.5) / resolution;
  p.z = 0;
  return p;
}

math::geometry::LineSegment Grid::transformToPixel(const math::geometry::LineSegment & line) const
{
  return math::geometry::LineSegment(
    transformToPixel(line.start_point), transformToPixel(line.end_point));
}

math::geometry::LineSegment Grid::getInvisibleRay(
  const geometry_msgs::msg::Point & point_on_polygon) const
{
  return math::geometry::LineSegment(
    point_on_polygon, math::geometry::LineSegment(origin_.position, point_on_polygon).get2DVector(),
    getDiagonalLength());
}

std::vector<math::geometry::LineSegment> Grid::getInvisibleRay(
  const std::vector<geometry_msgs::msg::Point> & points) const
{
  std::vector<math::geometry::LineSegment> ret = {};
  for (const auto & point : points) {
    ret.emplace_back(getInvisibleRay(point));
  }
  return ret;
}

std::vector<math::geometry::LineSegment> Grid::getRayToGridCorner() const
{
  geometry_msgs::msg::Point left_up;
  left_up.x = static_cast<double>(width) * resolution * 0.5;
  left_up.y = static_cast<double>(height) * resolution * 0.5;
  left_up = transformToWorld(left_up);
  geometry_msgs::msg::Point left_down;
  left_down.x = static_cast<double>(width) * resolution * 0.5;
  left_down.y = -static_cast<double>(height) * resolution * 0.5;
  left_down = transformToWorld(left_down);
  geometry_msgs::msg::Point right_up;
  right_up.x = -static_cast<double>(width) * resolution * 0.5;
  right_up.y = static_cast<double>(height) * resolution * 0.5;
  right_up = transformToWorld(right_up);
  geometry_msgs::msg::Point right_down;
  right_down.x = -static_cast<double>(width) * resolution * 0.5;
  right_down.y = -static_cast<double>(height) * resolution * 0.5;
  right_down = transformToWorld(right_down);
  return {
    math::geometry::LineSegment(origin_.position, left_up),
    math::geometry::LineSegment(origin_.position, left_down),
    math::geometry::LineSegment(origin_.position, right_down),
    math::geometry::LineSegment(origin_.position, right_up)};
}

size_t Grid::getIndex(size_t row, size_t col) const { return width * col + row; }
size_t Grid::getNextRowIndex(size_t row, size_t col) const { return width * col + (row + 1); }
size_t Grid::getNextColIndex(size_t row, size_t col) const { return width * (col + 1) + row; }
size_t Grid::getPreviousRowIndex(size_t row, size_t col) const { return width * col + (row - 1); }
size_t Grid::getPreviousColIndex(size_t row, size_t col) const { return width * (col - 1) + row; }
bool Grid::indexExist(size_t index) const
{
  if (index <= (height * width - 1)) {
    return true;
  }
  return false;
}

std::vector<std::pair<size_t, size_t>> Grid::fillByIntersection(
  const math::geometry::LineSegment & line_segment, int8_t data)
{
  std::vector<std::pair<size_t, size_t>> ret;
  const auto line_segment_pixel = transformToPixel(transformToGrid(line_segment));
  int start_row = std::floor(line_segment_pixel.start_point.x);
  int start_col = std::floor(line_segment_pixel.start_point.y);
  int end_row = std::floor(line_segment_pixel.end_point.x);
  int end_col = std::floor(line_segment_pixel.end_point.y);
  if (start_row == end_row) {
    for (int col = start_col; col <= end_col; col++) {
      if (fillByRowCol(start_row, col, data)) {
        ret.emplace_back(std::pair<size_t, size_t>({start_row, col}));
      }
    }
    sortAndUnique(ret);
    return ret;
  }
  if (start_col == end_col) {
    for (int row = start_row; row <= end_row; row++) {
      if (fillByRowCol(row, start_col, data)) {
        ret.emplace_back(std::pair<size_t, size_t>({row, start_col}));
      }
    }
    sortAndUnique(ret);
    return ret;
  }
  for (int row = std::min(start_row, end_row) + 1; row < std::max(start_row, end_row) + 1; row++) {
    if (0 <= row && row < static_cast<int>(width)) {
      int col = std::floor(
        line_segment_pixel.getSlope() * static_cast<double>(row) +
        line_segment_pixel.getIntercept());
      if (0 <= col && col < static_cast<int>(height)) {
        if (fillByRowCol(row, col, data)) {
          ret.emplace_back(std::pair<size_t, size_t>({row, col}));
        }
        if (row != std::max(start_row, end_row)) {
          if (fillByRowCol(row - 1, col, data)) {
            ret.emplace_back(std::pair<size_t, size_t>({row - 1, col}));
          }
        }
      }
    }
  }
  for (int col = std::min(start_col, end_col) + 1; col < std::max(start_col, end_col) + 1; col++) {
    if (0 <= col && col < static_cast<int>(height)) {
      int row = std::floor(
        (static_cast<double>(col) - line_segment_pixel.getIntercept()) /
        line_segment_pixel.getSlope());
      if (0 <= row && row < static_cast<int>(width)) {
        if (fillByRowCol(row, col, data)) {
          ret.emplace_back(std::pair<size_t, size_t>({row, col}));
        }
        if (col != std::max(start_col, end_col)) {
          if (fillByRowCol(row, col - 1, data)) {
            ret.emplace_back(std::pair<size_t, size_t>({row, col - 1}));
          }
        }
      }
    }
  }
  sortAndUnique(ret);
  return ret;
}

std::vector<std::pair<size_t, size_t>> Grid::fillByIntersection(
  const std::vector<math::geometry::LineSegment> & line_segments, int8_t data)
{
  std::vector<std::pair<size_t, size_t>> filled_cells = {};
  for (const auto & line : line_segments) {
    append(filled_cells, fillByIntersection(line, data));
  }
  return filled_cells;
}

std::vector<std::pair<size_t, size_t>> Grid::fillInside(
  const std::vector<std::pair<size_t, size_t>> & row_and_cols, int8_t data)
{
  auto ret = std::vector<std::pair<size_t, size_t>>();

  auto group_by_row = std::map<size_t, std::vector<size_t>>();
  for (const auto [row, col] : row_and_cols) {
    group_by_row[row].emplace_back(col);
  }
  for (const auto & [row, cols] : group_by_row) {
    if (cols.size() <= 1) continue;
    auto [min_col_itr, max_col_itr] = std::minmax_element(cols.begin(), cols.end());
    for (auto col = *min_col_itr; col <= *max_col_itr; col++) {
      if (fillByRowCol(row, col, data)) {
        ret.emplace_back(row, col);
      }
    }
  }

  auto group_by_col = std::map<size_t, std::vector<size_t>>();
  for (const auto [row, col] : row_and_cols) {
    group_by_col[col].emplace_back(row);
  }
  for (const auto & [col, rows] : group_by_col) {
    if (rows.size() <= 1) continue;
    auto [min_row_itr, max_row_itr] = std::minmax_element(rows.begin(), rows.end());
    for (auto row = *min_row_itr; row <= *max_row_itr; row++) {
      if (fillByRowCol(row, col, data)) {
        ret.emplace_back(row, col);
      }
    }
  }

  return ret;
}

std::vector<std::pair<size_t, size_t>> Grid::filterByRow(
  const std::vector<std::pair<size_t, size_t>> & row_and_cols, size_t row) const
{
  std::vector<std::pair<size_t, size_t>> filtered;
  for (const auto & row_and_col : row_and_cols) {
    if (row_and_col.first == row) {
      filtered.emplace_back(row_and_col);
    }
  }
  sortAndUnique(filtered);
  return filtered;
}

std::vector<std::pair<size_t, size_t>> Grid::filterByCol(
  const std::vector<std::pair<size_t, size_t>> & row_and_cols, size_t col) const
{
  std::vector<std::pair<size_t, size_t>> filtered;
  for (const auto & row_and_col : row_and_cols) {
    if (row_and_col.second == col) {
      filtered.emplace_back(row_and_col);
    }
  }
  sortAndUnique(filtered);
  return filtered;
}

std::vector<math::geometry::LineSegment> Grid::filterByIntersection(
  const std::vector<math::geometry::LineSegment> & source_lines,
  const std::vector<math::geometry::LineSegment> & filter_lines) const
{
  std::vector<math::geometry::LineSegment> filtered_lines;
  for (const auto & source_line : source_lines) {
    for (const auto & filter_line : filter_lines) {
      if (source_line.getIntersection2D(filter_line)) {
        filtered_lines.emplace_back(source_line);
        break;
      }
    }
  }
  return filtered_lines;
}

std::vector<size_t> Grid::getRows(const std::vector<std::pair<size_t, size_t>> & row_and_cols) const
{
  std::vector<size_t> ret;
  for (const auto & row_and_col : row_and_cols) {
    ret.emplace_back(row_and_col.first);
  }
  sortAndUnique(ret);
  return ret;
}

std::vector<size_t> Grid::getCols(const std::vector<std::pair<size_t, size_t>> & row_and_cols) const
{
  std::vector<size_t> ret;
  for (const auto & row_and_col : row_and_cols) {
    ret.emplace_back(row_and_col.second);
  }
  sortAndUnique(ret);
  return ret;
}

void Grid::addPrimitive(const std::unique_ptr<primitives::Primitive> & primitive)
{
  const auto hull = primitive->get2DConvexHull();
  const auto line_segments_on_hull = math::geometry::getLineSegments(hull);
  std::vector<math::geometry::LineSegment> rays_to_grid_corner = {};
  for (const auto & ray : filterByIntersection(getRayToGridCorner(), line_segments_on_hull)) {
    for (const auto & line_segment : line_segments_on_hull) {
      const auto intersection = ray.getIntersection2D(line_segment);
      if (intersection) {
        rays_to_grid_corner.emplace_back(
          math::geometry::LineSegment(intersection.get(), ray.get2DVector(), getDiagonalLength()));
      }
    }
  }
  fillInside(
    fillByIntersection(
      concat(line_segments_on_hull, concat(getInvisibleRay(hull), rays_to_grid_corner)),
      invisible_cost),
    invisible_cost);
  fillInside(fillByIntersection(line_segments_on_hull, occupied_cost), occupied_cost);
}

const std::vector<int8_t> &Grid::getData()
{
  return values_;
}

bool Grid::fillByIndex(size_t index, int8_t data)
{
  if (index < values_.size()) {
    values_[index] = data;
    return true;
  }
  return false;
}

bool Grid::fillByRowCol(size_t row, size_t col, int8_t data)
{
  if (row >= width) {
    return false;
  }
  if (col >= height) {
    return false;
  }
  return fillByIndex(getIndex(row, col), data);
}

void Grid::fillByRow(size_t row, int8_t data)
{
  if (row < height) {
    for (size_t col = 0; col < width; col++) {
      fillByRowCol(row, col, data);
    }
  }
}

void Grid::fillByCol(size_t col, int8_t data)
{
  if (col < width) {
    for (size_t row = 0; row < height; row++) {
      fillByRowCol(row, col, data);
    }
  }
}

void Grid::updateOrigin(const geometry_msgs::msg::Pose & origin)
{
  origin_ = origin;
  std::fill(values_.begin(), values_.end(), int8_t(0));
}

}  // namespace simple_sensor_simulator
