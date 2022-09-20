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
  using MarkerCounterType = int16_t;
  using MarkerGridType = std::vector<MarkerCounterType>;
  using OccupancyGridType = std::vector<int8_t>;
  using PointType = geometry_msgs::msg::Point;
  using PoseType = geometry_msgs::msg::Pose;
  using PrimitiveType = primitives::Primitive;
  using PolygonType = std::vector<PointType>;

  Grid(
    double resolution, size_t height, size_t width, int8_t occupied_cost = 100,
    int8_t invisible_cost = 50);

  const double resolution;
  const size_t height;
  const size_t width;
  const int8_t occupied_cost;
  const int8_t invisible_cost;

  /**
   * @brief Mark invisible area and occpuied area of primitive
   * @param primitive
   */
  auto add(const PrimitiveType & primitive) -> void;

  /**
   * @brief Reset all internal state
   * @param origin
   */
  auto reset(const PoseType & origin) -> void;

  /**
   * @brief Construct occupancy grid
   */
  auto construct() -> void;

  /**
   * @return Constructed occupancy grid
   */
  auto get() const -> const OccupancyGridType &;

private:
  /**
   * @brief Grid origin in world coordinate
   */
  PoseType origin_;

  /**
   * @brief The number of added primitives
   */
  MarkerCounterType primitive_count_ = 0;

  /**
   * @brief A vector of occupied area
   * @note This vector is declared as a member in order to reuse allocated memory
   */
  MarkerGridType occupied_grid_;

  /**
   * @brief A vector of invisible area
   * @note This vector is declared as a member in order to reuse allocated memory
   */
  MarkerGridType invisible_grid_;

  /**
   * @brief A vector of grid values
   * @note This vector is declared as a member in order to reuse allocated memory
   */
  OccupancyGridType values_;

  /**
   * @brief Vectors to hold min or max column of rasterized polygon
   * @note These vectors are declared as members in order to reuse allocated memory
   */
  std::vector<ssize_t> mincols_, maxcols_;

  /**
   * @brief Traverse grid cells from start to end
   * @param start
   * @param end
   * @param f a funciton object which takes cell coordinate
   */
  template <class F>
  inline auto traverse(const PointType & start, const PointType & end, const F & f)
    -> void
  {
    // A Fast Voxel Traversal Algorithm for Ray Tracing
    // https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.42.3443&rep=rep1&type=pdf

    double vx = end.x - start.x;
    double vy = end.y - start.y;

    ssize_t step_x = ssize_t(std::copysign(1.0, vx));
    ssize_t step_y = ssize_t(std::copysign(1.0, vy));

    double tdx = step_x / vx;
    double tdy = step_y / vy;

    double tx = vx > 0 ? std::ceil(start.x) : std::floor(start.x);
    double ty = vy > 0 ? std::ceil(start.y) : std::floor(start.y);

    tx = vx != 0 ? (tx - start.x) / vx : tdx;
    ty = vy != 0 ? (ty - start.y) / vy : tdy;

    ssize_t x = ssize_t(start.x);
    ssize_t y = ssize_t(start.y);

    while (tx <= 1.0 || ty <= 1.0) {
      f(x, y);
      if (tx < ty) {
        tx += tdx;
        x += step_x;
      } else {
        ty += tdy;
        y += step_y;
      }
    }
  }

  /**
   * @brief Mark grid area of convex hull
   * @param grid Grid to be marked
   * @param convex_hull Convex hull to mark
   */
  inline auto markConvexHull(
    MarkerGridType & grid, const PolygonType & convex_hull) -> void;

  /**
   * @brief Convert point in world coordinate to point in grid cooridnate
   * @param world_point
   * @return Point in grid coordinate
   */
  inline auto transformToGrid(const PointType & world_point) const
    -> PointType;

  /**
   * @brief Rasterize point in grid coordinate
   * @param grid_point
   * @return Rasterized point
   */
  inline auto transformToPixel(const PointType & grid_point) const
    -> PointType;

  /**
   * @brief An utility function to construct a Point
   * @param x
   * @param y
   * @param z
   */
  inline auto constructPoint(double x, double y, double z) const -> PointType;

  /**
   * @brief Search minmax angle point from a polygon
   * @param polygon
   * @return Iterator pair of minmax angle point
   * @note This function use [-pi/2, pi/2] as angle interval by default. If polygon crosses
   *       this interval, this function automatically switch to use [0, 2pi] as angle interval.
   */
  inline auto minmaxAnglePoint(const PolygonType & polygon) const;

  /**
   * @brief Construct a convex hull of the area occupied with primitive
   * @param primitive
   * @return Convex hull polygon
   */
  inline auto constructOccupiedConvexHull(const PrimitiveType & primitive) const
    -> PolygonType;

  /**
   * @brief Construct a convex hull of the area made invisible by the occupied area
   * @param occupied_convex_hull Convex hull of occupied area
   * @return Convex hull polygon
   */
  inline auto constructInvisibleConvexHull(
    const PolygonType & occupied_convex_hull) const
    -> PolygonType;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_
