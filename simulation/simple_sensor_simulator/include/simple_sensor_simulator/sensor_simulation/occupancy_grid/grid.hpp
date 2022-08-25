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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_

#include <boost/optional.hpp>
#include <geometry/polygon/line_segment.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
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
   * @brief Get all cell values
   * @return cell values
   */
  const std::vector<int8_t> & getData();

  /**
   * @brief Reset origin and all cell values
   * @note Use this function to reuse already allocated memory
   */
  void reset(const geometry_msgs::msg::Pose & origin);

private:
  /**
   * @brief origin
   * @note Grid treats Ego's origin as its origin
   */
  geometry_msgs::msg::Pose origin_;

  /**
   * @brief a vector which contains cell values
   * @note Grid access this 1d vector by calculating an index from a 2d grid coordinate
   */
  std::vector<int8_t> values_;

  /**
   * @brief Update value of cell locate at (`row`, `col`) to `data`
   * @return true if cell value is updated otherwise false
   */
  bool fillByRowCol(size_t row, size_t col, int8_t data);

  /**
   * @brief Update value of cells intersect with `line_segment` to `data`
   * @param [out] ret coordinates of filled cells
   */
  void fillByIntersection(
    const math::geometry::LineSegment & line_segment, int8_t data,
    std::vector<std::pair<size_t, size_t>> & ret);

  /**
   * @brief Update value of cells surrounded by `row_and_cols` to `data`
   */
  void fillInside(const std::vector<std::pair<size_t, size_t>> & row_and_cols, int8_t data);

  /**
   * @brief Convert point in grid coordinate to point in world cooridnate
   * @return Point in world coordinate
   */
  geometry_msgs::msg::Point transformToWorld(const geometry_msgs::msg::Point & grid_point) const;

  /**
   * @brief Convert point in world coordinate to point in grid cooridnate
   * @return Point in grid coordinate
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
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_
