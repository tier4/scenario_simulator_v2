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
   * @brief Traverse cells along with `line`
   * @param [out] ret coordinates of filled cells
   */
  template<class F>
  void traverseLineSegment(const math::geometry::LineSegment & line, const F &f)
  {
    // A Fast Voxel Traversal Algorithm for Ray Tracing
    // https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.42.3443&rep=rep1&type=pdf

    auto start = transformToPixel(line.start_point);
    auto end = transformToPixel(line.end_point);

    double start_x = start.x;
    double start_y = start.y;
    double end_x = end.x;
    double end_y = end.y;

    double vx = end_x - start_x;
    double vy = end_y - start_y;

    if (start_x < 0 || start_x > width) {
      double dx = start_x < 0 ? -start_x : width - start_x;
      double dy = vy * dx / vx;

      start_x += dx;
      start_y += dy;
    }

    if (start_y < 0 || start_y > height) {
      double dy = start_y < 0 ? -start_y : height - start_y;
      double dx = vx * dy / vy;

      start_x += dx;
      start_y += dy;
    }

    if (end_x < 0 || end_x > width) {
      double dx = end_x < 0 ? -end_x : width - end_x;
      double dy = vy * dx / vx;

      end_x += dx;
      end_y += dy;
    }

    if (end_y < 0 || end_y > height) {
      double dy = end_y < 0 ? -end_y : height - end_y;
      double dx = vx * dy / vy;

      end_x += dx;
      end_y += dy;
    }

    vx = end_x - start_x;
    vy = end_y - start_y;

    ssize_t col = static_cast<ssize_t>(start_x);
    ssize_t row = static_cast<ssize_t>(start_y);

    ssize_t step_col = static_cast<ssize_t>(std::copysign(1.0, vx));
    ssize_t step_row = static_cast<ssize_t>(std::copysign(1.0, vy));

    double tdx = 1.0 / std::abs(vx);
    double tdy = 1.0 / std::abs(vy);

    double tx = vx > 0 ? (std::ceil(start_x) - start_x) * tdx : (start_x - std::floor(start_x)) * tdx;
    double ty = vy > 0 ? (std::ceil(start_y) - start_y) * tdy : (start_y - std::floor(start_y)) * tdy;

    while (tx <= 1.0 || ty <= 1.0) {
      f(col, row);
      if (tx < ty) {
        tx += tdx;
        col += step_col;
      } else {
        ty += tdy;
        row += step_row;
      }
    }
  }

  /**
   * @brief Update value of cells surrounded by `segments` to `data`
   */
  void fillInside(const std::vector<math::geometry::LineSegment> & segments, int8_t data);

  /**
   * @brief Convert point in world coordinate to point in grid cooridnate
   * @return Point in grid coordinate
   */
  geometry_msgs::msg::Point transformToGrid(const geometry_msgs::msg::Point & world_point) const;

  /**
   * @brief Digitize point in grid coordinate
   * @return Digitized point
   */
  geometry_msgs::msg::Point transformToPixel(const geometry_msgs::msg::Point & grid_point) const;

};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_
