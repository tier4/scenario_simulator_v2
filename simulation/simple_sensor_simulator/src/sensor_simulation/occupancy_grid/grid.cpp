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
{
}

geometry_msgs::msg::Point Grid::transformToGrid(const geometry_msgs::msg::Point & world_point) const
{
  auto conj = quaternion_operation::conjugate(origin_.orientation);
  auto rot = quaternion_operation::getRotationMatrix(conj);
  auto p = Eigen::Vector3d(world_point.x, world_point.y, world_point.z);
  auto q = Eigen::Vector3d(origin_.position.x, origin_.position.y, origin_.position.z);
  p = rot * p - q;

  geometry_msgs::msg::Point ret;
  ret.x = p(0), ret.y = p(1), ret.z = p(2);
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

void Grid::fillInside(const std::vector<math::geometry::LineSegment> & segments, int8_t data)
{
  auto group_by_row = std::vector<std::vector<size_t>>(height);
  auto group_by_col = std::vector<std::vector<size_t>>(width);
  for (auto & segment : segments) {
    traverseLineSegment(segment, [&](size_t col, size_t row) {
      group_by_row[row].emplace_back(col);
      group_by_col[col].emplace_back(row);
    });
  }

  for (size_t row = 0; row < height; ++row) {
    if (const auto & cols = group_by_row[row]; cols.size() > 1) {
      auto [min_col_itr, max_col_itr] = std::minmax_element(cols.begin(), cols.end());
      for (auto col = *min_col_itr; col <= *max_col_itr; ++col) {
        values_[width * row + col] = data;
      }
    }
  }

  for (size_t col = 0; col < width; ++col) {
    if (const auto & rows = group_by_col[col]; rows.size() > 1) {
      auto [min_row_itr, max_row_itr] = std::minmax_element(rows.begin(), rows.end());
      for (auto row = *min_row_itr; row <= *max_row_itr; ++row) {
        values_[width * row + col] = data;
      }
    }
  }
}

void Grid::addPrimitive(const std::unique_ptr<primitives::Primitive> & primitive)
{
  using math::geometry::LineSegment;
  using geometry_msgs::msg::Point;

  auto hull = primitive->get2DConvexHull();
  for (auto &p : hull) {
    p = transformToGrid(p);
  }
  const auto occupied_edges = math::geometry::getLineSegments(hull);

  auto invisible_edges = occupied_edges;
  {
    auto corners = std::vector<Point>(4);
    {
      double half_rw = static_cast<double>(width) * resolution / 2;
      double half_rh = static_cast<double>(height) * resolution / 2;

      // enumerate corner coordinates
      corners[0].x = half_rw, corners[0].y = half_rh;
      corners[1].x = -half_rw, corners[1].y = half_rh;
      corners[2].x = half_rw, corners[2].y = -half_rh;
      corners[3].x = -half_rw, corners[3].y = -half_rh;
    }

    auto diagonal_length = std::hypot(width * resolution, height * resolution);

    // add rays through grid corners
    for (const auto & corner : corners) {
      auto ray = LineSegment(Point(), corner);
      for (const auto & line_segment : occupied_edges) {
        if (const auto intersection = ray.getIntersection2D(line_segment)) {
          invisible_edges.emplace_back(intersection.get(), ray.get2DVector(), diagonal_length);
        }
      }
    }
    // add rays through hull points
    for (const auto & point : hull) {
      auto ray = LineSegment(Point(), point);
      invisible_edges.emplace_back(point, ray.get2DVector(), diagonal_length);
    }
  }

  // fill invisible area
  fillInside(invisible_edges, invisible_cost);

  // fill occupied area
  fillInside(occupied_edges, occupied_cost);
}

const std::vector<int8_t> & Grid::getData() { return values_; }

void Grid::reset(const geometry_msgs::msg::Pose & origin)
{
  origin_ = origin;
  values_.assign(values_.size(), 0);
}

}  // namespace simple_sensor_simulator
