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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_CELL_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_CELL_HPP_

#include <array>
#include <boost/optional.hpp>
#include <geometry/polygon/line_segment.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <vector>

namespace simple_sensor_simulator
{
std::vector<geometry_msgs::msg::Point> getIntersection2D(
  const std::vector<math::geometry::LineSegment> & lines);

class GridCell
{
public:
  /**
   * @defgroup Public members
   * @note These members are intentionally kept non-const for performance reason
   */
  /*@{*/
  geometry_msgs::msg::Pose origin;
  double size;
  size_t index;
  size_t row;
  size_t col;
  /*@}*/

  /**
   * @brief Check if this cell contains point `p`
   * @return true if this cell contains point otherwise false
   */
  bool contains(const geometry_msgs::msg::Point & p) const;

  /**
   * @brief Check if this cell contains any of `points`
   * @return true if this cell contains any of `points` otherwise false
   */
  bool contains(const std::vector<geometry_msgs::msg::Point> & points) const;

  /**
   * @brief Get data contained in this cell
   * @return data value
   */
  int8_t getData() const;

  /**
   * @brief Set data contained in this cell
   * @return data value
   */
  void setData(int8_t data);

  /**
   * @brief get top left corner
   * @return top left corner
   */
  geometry_msgs::msg::Point getLeftUpPoint() const;

  /**
   * @brief get bottom left corner
   * @return bottom left corner
   */
  geometry_msgs::msg::Point getLeftDownPoint() const;

  /**
   * @brief get top right corner
   * @return top right corner
   */
  geometry_msgs::msg::Point getRightUpPoint() const;

  /**
   * @brief get bottom right corner
   * @return bottom right corner
   */
  geometry_msgs::msg::Point getRightDownPoint() const;

private:
  /**
   * @brief a value kept in this cell
   */
  int8_t data_ = 0;

  /**
   * @brief Convert point in grid to a point in world coordinate
   * @return point in world coordinate
   */
  geometry_msgs::msg::Point transformToWorld(const geometry_msgs::msg::Point & point) const;

  /**
   * @brief get a polygon of this cell
   * @return a polygon of this cell in vector form
   */
  std::vector<geometry_msgs::msg::Point> getPolygon() const;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_CELL_HPP_
