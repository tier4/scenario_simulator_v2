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
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid_cell.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <vector>

namespace simple_sensor_simulator
{
class Grid
{
public:
  Grid(
    const geometry_msgs::msg::Pose & origin, double resolution, size_t height, size_t width,
    int8_t occupied_cost = 100, int8_t invisible_cost = 50);
  const double resolution;
  const size_t height;
  const size_t width;
  const geometry_msgs::msg::Pose origin;
  const int8_t occupied_cost;
  const int8_t invisible_cost;
  void addPrimitive(const std::unique_ptr<primitives::Primitive> & primitive);
  std::vector<int8_t> getData();

private:
  std::vector<GridCell> grid_cells_;
  bool fillByIndex(size_t index, int8_t data);
  void fillByRow(size_t row, int8_t data);
  void fillByCol(size_t col, int8_t data);
  bool fillByRowCol(size_t row, size_t col, int8_t data);
  std::vector<std::pair<size_t, size_t>> fillByIntersection(
    const LineSegment & line_segment, int8_t data);
  std::vector<std::pair<size_t, size_t>> fillByIntersection(
    const std::vector<LineSegment> & line_segments, int8_t data);
  std::vector<std::pair<size_t, size_t>> fillInside(
    const std::vector<std::pair<size_t, size_t>> & row_and_cols, int8_t data);
  size_t getIndex(size_t row, size_t col) const;
  size_t getNextRowIndex(size_t row, size_t col) const;
  size_t getNextColIndex(size_t row, size_t col) const;
  size_t getPreviousRowIndex(size_t row, size_t col) const;
  size_t getPreviousColIndex(size_t row, size_t col) const;
  std::vector<std::pair<size_t, size_t>> filterByRow(
    const std::vector<std::pair<size_t, size_t>> & row_and_cols, size_t row) const;
  std::vector<std::pair<size_t, size_t>> filterByCol(
    const std::vector<std::pair<size_t, size_t>> & row_and_cols, size_t col) const;
  std::vector<LineSegment> filterByIntersection(
    const std::vector<LineSegment> & source_lines,
    const std::vector<LineSegment> & fillter_lines) const;
  std::vector<size_t> getRows(const std::vector<std::pair<size_t, size_t>> & row_and_cols) const;
  std::vector<size_t> getCols(const std::vector<std::pair<size_t, size_t>> & row_and_cols) const;
  bool indexExist(size_t index) const;
  std::vector<GridCell> getAllCells() const;
  geometry_msgs::msg::Point transformToWorld(const geometry_msgs::msg::Point & grid_point) const;
  geometry_msgs::msg::Point transformToGrid(const geometry_msgs::msg::Point & world_point) const;
  LineSegment transformToGrid(const LineSegment & line) const;
  geometry_msgs::msg::Point transformToPixel(const geometry_msgs::msg::Point & grid_point) const;
  LineSegment transformToPixel(const LineSegment & line) const;
  LineSegment getInvisibleRay(const geometry_msgs::msg::Point & point_on_polygon) const;
  std::vector<LineSegment> getRayToGridCorner();
  std::vector<LineSegment> getInvisibleRay(
    const std::vector<geometry_msgs::msg::Point> & points) const;
  double getDiagonalLength() const;
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