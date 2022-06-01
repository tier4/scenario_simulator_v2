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
Grid::Grid(
  const geometry_msgs::msg::Pose & origin, double resolution, size_t height, size_t width,
  int8_t occupied_cost, int8_t invisible_cost)
: resolution(resolution),
  height(height),
  width(width),
  origin(origin),
  occupied_cost(occupied_cost),
  invisible_cost(invisible_cost),
  grid_cells_(getAllCells())
{
}

double Grid::getDiagonalLength() const { return std::hypot(width, height) * resolution; }

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

geometry_msgs::msg::Point Grid::transformToWorld(const geometry_msgs::msg::Point & grid_point) const
{
  auto mat = quaternion_operation::getRotationMatrix(origin.orientation);
  Eigen::VectorXd p(3);
  p(0) = grid_point.x;
  p(1) = grid_point.y;
  p(2) = grid_point.z;
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

LineSegment Grid::getInvisibleRay(const geometry_msgs::msg::Point & point_on_polygon) const
{
  return LineSegment(
    point_on_polygon, LineSegment(origin.position, point_on_polygon).get2DVector(),
    getDiagonalLength());
}

std::vector<LineSegment> Grid::getInvisibleRay(
  const std::vector<geometry_msgs::msg::Point> & points) const
{
  std::vector<LineSegment> ret = {};
  for (const auto & point : points) {
    ret.emplace_back(getInvisibleRay(point));
  }
  return ret;
}

std::vector<LineSegment> Grid::getRayToGridCorner()
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
    LineSegment(origin.position, left_up), LineSegment(origin.position, left_down),
    LineSegment(origin.position, right_down), LineSegment(origin.position, right_up)};
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
  std::vector<std::pair<size_t, size_t>> ret;
  const auto rows = getRows(row_and_cols);
  for (const auto & row : rows) {
    const auto cells_in_row = filterByRow(row_and_cols, row);
    if (cells_in_row.size() > 1) {
      for (auto col = cells_in_row[0].second; col <= cells_in_row[cells_in_row.size() - 1].second;
           col++) {
        if (fillByRowCol(row, col, data)) {
          ret.emplace_back(std::pair<size_t, size_t>(row, col));
        }
      }
    }
  }
  const auto cols = getCols(row_and_cols);
  for (const auto & col : cols) {
    const auto cells_in_col = filterByCol(row_and_cols, col);
    if (cells_in_col.size() > 1) {
      for (auto row = cells_in_col[0].first; row <= cells_in_col[cells_in_col.size() - 1].first;
           row++) {
        if (fillByRowCol(row, col, data)) {
          ret.emplace_back(std::pair<size_t, size_t>(row, col));
        }
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

std::vector<LineSegment> Grid::filterByIntersection(
  const std::vector<LineSegment> & source_lines,
  const std::vector<LineSegment> & filter_lines) const
{
  std::vector<LineSegment> filtered_lines;
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
  const auto line_segments_on_hull = getLineSegments(hull);
  std::vector<LineSegment> rays_to_grid_corner = {};
  for (const auto & ray : filterByIntersection(getRayToGridCorner(), line_segments_on_hull)) {
    for (const auto & line_segment : line_segments_on_hull) {
      const auto intersection = ray.getIntersection2D(line_segment);
      if (intersection) {
        rays_to_grid_corner.emplace_back(
          LineSegment(intersection.get(), ray.get2DVector(), getDiagonalLength()));
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
}  // namespace simple_sensor_simulator
