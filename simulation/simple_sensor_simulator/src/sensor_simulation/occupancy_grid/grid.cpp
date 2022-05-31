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
Grid::Grid(const geometry_msgs::msg::Pose & origin, double resolution, size_t height, size_t width)
: resolution(resolution), height(height), width(width), origin(origin), grid_cells_(getAllCells())
{
}

geometry_msgs::msg::Point Grid::transformToGrid(const geometry_msgs::msg::Point & world_point) const
{
  auto mat =
    quaternion_operation::getRotationMatrix(quaternion_operation::conjugate(origin.orientation));
  Eigen::VectorXd p(3);
  p(0) = world_point.x;
  p(1) = world_point.y;
  p(2) = world_point.z;
  p = mat * p;
  p(0) = p(0) - origin.position.x;
  p(1) = p(1) - origin.position.y;
  p(2) = p(2) - origin.position.z;
  geometry_msgs::msg::Point ret;
  ret.x = p(0);
  ret.y = p(1);
  ret.z = p(2);
  return ret;
}

LineSegment Grid::transformToGrid(const LineSegment & line) const
{
  return LineSegment(transformToGrid(line.start_point), transformToGrid(line.end_point));
}

geometry_msgs::msg::Point Grid::transformToPixel(const geometry_msgs::msg::Point & grid_point) const
{
  geometry_msgs::msg::Point p;
  p.x = (grid_point.x + height * resolution * 0.5) / resolution;
  p.y = (grid_point.y + width * resolution * 0.5) / resolution;
  p.z = 0;
  return p;
}

LineSegment Grid::transformToPixel(const LineSegment & line) const
{
  return LineSegment(transformToPixel(line.start_point), transformToPixel(line.end_point));
}

std::vector<GridCell> Grid::getAllCells() const
{
  std::vector<GridCell> ret;
  for (size_t x_index = 0; x_index < height; x_index++) {
    for (size_t y_index = 0; y_index < width; y_index++) {
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
  const LineSegment & line_segment, int8_t data)
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
        ret.emplace_back(std::pair<size_t, size_t>({start_col, row}));
      }
    }
    sortAndUnique(ret);
    return ret;
  }
  if (fillByRowCol(start_row, start_col, data)) {
    ret.emplace_back(std::pair<size_t, size_t>({start_row, start_col}));
  }
  if (fillByRowCol(end_row, end_col, data)) {
    ret.emplace_back(std::pair<size_t, size_t>({end_row, end_col}));
  }
  for (int row = std::min(start_row, end_row) + 1; row < std::max(start_row, end_row) + 1; row++) {
    int col = std::floor(
      line_segment_pixel.getSlope() * static_cast<double>(row) + line_segment_pixel.getIntercept());
    if (fillByRowCol(row, col, data)) {
      ret.emplace_back(std::pair<size_t, size_t>({row, col}));
    }
    if (row != std::max(start_row, end_row)) {
      if (fillByRowCol(row - 1, col, data)) {
        ret.emplace_back(std::pair<size_t, size_t>({row - 1, col}));
      }
    }
  }
  for (int col = std::min(start_col, end_col) + 1; col < std::max(start_col, end_col) + 1; col++) {
    int row = std::floor(
      (static_cast<double>(col) - line_segment_pixel.getIntercept()) /
      line_segment_pixel.getSlope());
    if (fillByRowCol(row, col, data)) {
      ret.emplace_back(std::pair<size_t, size_t>({row, col}));
    }
    if (col != std::max(start_col, end_col)) {
      if (fillByRowCol(row, col - 1, data)) {
        ret.emplace_back(std::pair<size_t, size_t>({row, col - 1}));
      }
    }
  }
  sortAndUnique(ret);
  return ret;
}

std::vector<std::pair<size_t, size_t>> Grid::fillByIntersection(
  const std::vector<LineSegment> & line_segments, int8_t data)
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
  for (const auto & index : row_and_cols) {
  }
}

std::vector<std::pair<size_t, size_t>> Grid::filterByRow(
  const std::vector<std::pair<size_t, size_t>> & row_and_cols, size_t row) const
{
  std::vector<std::pair<size_t, size_t>> filtered;
  std::copy_if(
    row_and_cols.begin(), row_and_cols.end(), filtered.begin(),
    [&](std::pair<size_t, size_t> row_and_col) { return row_and_col.first == row; });
  sortAndUnique(filtered);
  return filtered;
}

std::vector<std::pair<size_t, size_t>> Grid::filterByCol(
  const std::vector<std::pair<size_t, size_t>> & row_and_cols, size_t col) const
{
  std::vector<std::pair<size_t, size_t>> filtered;
  std::copy_if(
    row_and_cols.begin(), row_and_cols.end(), filtered.begin(),
    [&](std::pair<size_t, size_t> row_and_col) { return row_and_col.second == col; });
  sortAndUnique(filtered);
  return filtered;
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
  const auto line_segments = getLineSegments(primitive->get2DConvexHull());
  fillByIntersection(line_segments, 100);
}

std::vector<int8_t> Grid::getData()
{
  std::vector<int8_t> data;
  for (const auto & cell : grid_cells_) {
    data.emplace_back(cell.getData());
  }
  return data;
}

bool Grid::fillByIndex(size_t index, int8_t data)
{
  if (index < grid_cells_.size()) {
    grid_cells_[index].setData(data);
    return true;
  }
  return false;
}

bool Grid::fillByRowCol(size_t row, size_t col, int8_t data)
{
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
}  // namespace simple_sensor_simulator
