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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_

#include <boost/optional.hpp>
#include <geometry/polygon/line_segment.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid_cell.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <vector>

namespace simple_sensor_simulator
{

/**
 * @brief grid
 */
class Grid
{
public:
  Grid(
    double resolution, size_t height, size_t width, int8_t occupied_cost = 100,
    int8_t invisible_cost = 50);
  const double resolution;
  const size_t height;
  const size_t width;
  const int8_t occupied_cost;
  const int8_t invisible_cost;

  /**
   * @brief Fill cells occupied by `primitive`
   */
  void addPrimitive(const std::unique_ptr<primitives::Primitive> & primitive);

  /**
   * @brief List all values from cells
   * @return cell values
   */
  std::vector<int8_t> getData();

  /**
   * @brief Update origin
   * @note This function also clear internal states
   */
  void updateOrigin(const geometry_msgs::msg::Pose & origin);

private:
  geometry_msgs::msg::Pose origin_;
  std::vector<GridCell> grid_cells_;

  /**
   * @brief Update cell value if index is in range
   * @return true if cell value is updated otherwise false
   */
  bool fillByIndex(size_t index, int8_t data);

  /**
   * @brief Update values of cells in `row` to `data`
   */
  void fillByRow(size_t row, int8_t data);

  /**
   * @brief Update values of cells in `col` to `data`
   */
  void fillByCol(size_t col, int8_t data);

  /**
   * @brief Update value of cell locate at (`row`, `col`) to `data`
   * @return true if cell value is updated otherwise false
   */
  bool fillByRowCol(size_t row, size_t col, int8_t data);

  /**
   * @brief Update value of cells intersect with `line_segment` to `data`
   * @return Vector of affected cell coordinates
   */
  std::vector<std::pair<size_t, size_t>> fillByIntersection(
    const math::geometry::LineSegment & line_segment, int8_t data);

  /**
   * @brief Update value of cells intersect with `line_segments` to `data`
   * @return Vector of affected cell coordinates
   */
  std::vector<std::pair<size_t, size_t>> fillByIntersection(
    const std::vector<math::geometry::LineSegment> & line_segments, int8_t data);

  /**
   * @brief Update value of cells surrounded by `row_and_cols` to `data`
   * @return Vector of affected cell coordinates
   */
  std::vector<std::pair<size_t, size_t>> fillInside(
    const std::vector<std::pair<size_t, size_t>> & row_and_cols, int8_t data);

  /**
   * @brief Caluculate index of coordinate
   * @return Index
   */
  size_t getIndex(size_t row, size_t col) const;

  /**
   * @brief Caluculate index of adjacent coordinate on next row
   * @return Index
   */
  size_t getNextRowIndex(size_t row, size_t col) const;

  /**
   * @brief Caluculate index of adjacent coordinate on next column
   * @return Index
   */
  size_t getNextColIndex(size_t row, size_t col) const;

  /**
   * @brief Caluculate index of adjacent coordinate on previous row
   * @return Index
   */
  size_t getPreviousRowIndex(size_t row, size_t col) const;

  /**
   * @brief Caluculate index of adjacent coordinate on previous column
   * @return Index
   */
  size_t getPreviousColIndex(size_t row, size_t col) const;

  /**
   * @brief List all cell coordinates on `row` from `row_and_cols`
   * @return Vector of cell coordinates  on `row` from `row_and_cols`
   */
  std::vector<std::pair<size_t, size_t>> filterByRow(
    const std::vector<std::pair<size_t, size_t>> & row_and_cols, size_t row) const;

  /**
   * @brief List all cell coordinates on `column` from `row_and_cols`
   * @return Vector of cell coordinates on `column` from `row_and_cols`
   */
  std::vector<std::pair<size_t, size_t>> filterByCol(
    const std::vector<std::pair<size_t, size_t>> & row_and_cols, size_t col) const;

  /**
   * @brief List all line segments intersect with any of `fileter_lines` from `source_lines`
   * @return Vector of line segments intersect with any of `fileter_lines` from `source_lines`
   */
  std::vector<math::geometry::LineSegment> filterByIntersection(
    const std::vector<math::geometry::LineSegment> & source_lines,
    const std::vector<math::geometry::LineSegment> & fillter_lines) const;

  /**
   * @brief List all rows in `row_and_cols`
   * @return vector of rows in `row_and_cols`
   */
  std::vector<size_t> getRows(const std::vector<std::pair<size_t, size_t>> & row_and_cols) const;

  /**
   * @brief List all columns in `row_and_cols`
   * @return vector of columns in `row_and_cols`
   */
  std::vector<size_t> getCols(const std::vector<std::pair<size_t, size_t>> & row_and_cols) const;

  /**
   * @brief Check if index is in grid
   * @return true if index is in grid otherwise false
   */
  bool indexExist(size_t index) const;

  /**
   * @brief Convert point in grid coordinate to point in world cooridnate
   * @return Point in orld coordinate
   */
  geometry_msgs::msg::Point transformToWorld(const geometry_msgs::msg::Point & grid_point) const;

  /**
   * @brief Convert point in world coordinate to point in grid cooridnate
   * @return Point in rid coordinate
   */
  geometry_msgs::msg::Point transformToGrid(const geometry_msgs::msg::Point & world_point) const;

  /**
   * @brief Convert line segment in world coordinate to line segment in grid cooridnate
   * @return Line segment in grid coordinate
   */
  math::geometry::LineSegment transformToGrid(const math::geometry::LineSegment & line) const;

  /**
   * @brief Digitize point in grid coordinate
   * @return Digitized point
   */
  geometry_msgs::msg::Point transformToPixel(const geometry_msgs::msg::Point & grid_point) const;

  /**
   * @brief Digitize line segment in grid coordinate
   * @return Digitized line segment
   */
  math::geometry::LineSegment transformToPixel(const math::geometry::LineSegment & line) const;

  /**
   * @brief Get pseudo half-open line segment through `point_on_polygon` from origin
   * @return Pseudo half-open line segment through `point_on_polygon` from origin
   */
  math::geometry::LineSegment getInvisibleRay(
    const geometry_msgs::msg::Point & point_on_polygon) const;

  /**
   * @brief Get pseudo half-open line segments through grid corners from origin
   * @return Pseudo half-open line segments through grid corners from origin
   */
  std::vector<math::geometry::LineSegment> getRayToGridCorner() const;

  /**
   * @brief Get pseudo half-open line segments through `points` from origin
   * @return Pseudo half-open line segment through `points` from origin
   */
  std::vector<math::geometry::LineSegment> getInvisibleRay(
    const std::vector<geometry_msgs::msg::Point> & points) const;

  /**
   * @brief Get length of grid diagonal
   */
  double getDiagonalLength() const;

  /**
   * @brief Re-calculate all cells
   */
  void updateAllCells();

  template <typename T>
  void sortAndUnique(std::vector<T> & data) const
  {
    std::sort(data.begin(), data.end());
    data.erase(std::unique(data.begin(), data.end()), data.end());
  }
  template <typename T>
  void append(std::vector<T> & v0, const std::vector<T> & v1) const
  {
    v0.insert(v0.end(), v1.begin(), v1.end());
  }
  template <typename T>
  std::vector<T> concat(const std::vector<T> & v0, const std::vector<T> & v1) const
  {
    std::vector<T> ret = v0;
    ret.insert(ret.end(), v1.begin(), v1.end());
    return ret;
  }
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_
