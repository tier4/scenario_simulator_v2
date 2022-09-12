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

math::geometry::LineSegment Grid::transformToGrid(const math::geometry::LineSegment & line) const
{
  return math::geometry::LineSegment(
    transformToGrid(line.start_point), transformToGrid(line.end_point));
}

geometry_msgs::msg::Point Grid::transformToWorld(const geometry_msgs::msg::Point & grid_point) const
{
  auto rot = quaternion_operation::getRotationMatrix(origin_.orientation);
  auto p = Eigen::Vector3d(grid_point.x, grid_point.y, grid_point.z);
  auto q = Eigen::Vector3d(origin_.position.x, origin_.position.y, origin_.position.z);
  p = rot * p + q;

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

math::geometry::LineSegment Grid::transformToPixel(const math::geometry::LineSegment & line) const
{
  return math::geometry::LineSegment(
    transformToPixel(line.start_point), transformToPixel(line.end_point));
}

void Grid::fillByIntersection(
  const math::geometry::LineSegment & line_segment, int8_t data,
  std::vector<std::pair<size_t, size_t>> & ret)
{
  const auto line_segment_pixel = transformToPixel(transformToGrid(line_segment));
  int start_row = std::floor(line_segment_pixel.start_point.x);
  int start_col = std::floor(line_segment_pixel.start_point.y);
  int end_row = std::floor(line_segment_pixel.end_point.x);
  int end_col = std::floor(line_segment_pixel.end_point.y);
  if (start_row == end_row) {
    for (int col = start_col; col <= end_col; col++) {
      if (fillByRowCol(start_row, col, data)) {
        ret.emplace_back(start_row, col);
      }
    }
  }
  if (start_col == end_col) {
    for (int row = start_row; row <= end_row; row++) {
      if (fillByRowCol(row, start_col, data)) {
        ret.emplace_back(row, start_col);
      }
    }
  }
  for (int row = std::min(start_row, end_row) + 1; row < std::max(start_row, end_row) + 1; row++) {
    if (0 <= row && row < static_cast<int>(width)) {
      int col = std::floor(
        line_segment_pixel.getSlope() * static_cast<double>(row) +
        line_segment_pixel.getIntercept());
      if (0 <= col && col < static_cast<int>(height)) {
        if (fillByRowCol(row, col, data)) {
          ret.emplace_back(row, col);
        }
        if (row != std::max(start_row, end_row)) {
          if (fillByRowCol(row - 1, col, data)) {
            ret.emplace_back(row - 1, col);
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
          ret.emplace_back(row, col);
        }
        if (col != std::max(start_col, end_col)) {
          if (fillByRowCol(row, col - 1, data)) {
            ret.emplace_back(row, col - 1);
          }
        }
      }
    }
  }
}

void Grid::fillInside(const std::vector<std::pair<size_t, size_t>> & row_and_cols, int8_t data)
{
  auto group_by_row = std::vector<std::vector<size_t>>(height);
  for (const auto [row, col] : row_and_cols) {
    group_by_row[row].emplace_back(col);
  }
  for (size_t row = 0; row < height; ++row) {
    if (const auto & cols = group_by_row[row]; cols.size() > 1) {
      auto [min_col_itr, max_col_itr] = std::minmax_element(cols.begin(), cols.end());
      for (auto col = *min_col_itr; col <= *max_col_itr; ++col) {
        fillByRowCol(row, col, data);
      }
    }
  }

  auto group_by_col = std::vector<std::vector<size_t>>(width);
  for (const auto [row, col] : row_and_cols) {
    group_by_col[col].emplace_back(row);
  }
  for (size_t col = 0; col < width; ++col) {
    if (const auto & rows = group_by_col[col]; rows.size() > 1) {
      auto [min_row_itr, max_row_itr] = std::minmax_element(rows.begin(), rows.end());
      for (auto row = *min_row_itr; row <= *max_row_itr; ++row) {
        fillByRowCol(row, col, data);
      }
    }
  }
}

void Grid::addPrimitive(const std::unique_ptr<primitives::Primitive> & primitive)
{
  const auto hull = primitive->get2DConvexHull();
  const auto occupied_edges = math::geometry::getLineSegments(hull);

  auto invisible_edges = occupied_edges;
  {
    auto corners = std::vector<geometry_msgs::msg::Point>(4);
    {
      // enumerate normalized corner coordinates
      corners[0].x = 0.5, corners[0].y = 0.5;
      corners[1].x = -0.5, corners[1].y = 0.5;
      corners[2].x = 0.5, corners[2].y = -0.5;
      corners[3].x = -0.5, corners[3].y = -0.5;

      for (auto & corner : corners) {
        // scale corner coordinates
        corner.x *= static_cast<double>(width) * resolution;
        corner.y *= static_cast<double>(height) * resolution;

        // transform to world coordinate system
        corner = transformToWorld(corner);
      }
    }

    auto diagonal_length = std::hypot(width, height) * resolution;

    // add rays through grid corners
    for (const auto & corner : corners) {
      auto ray = math::geometry::LineSegment(origin_.position, corner);
      for (const auto & line_segment : occupied_edges) {
        if (const auto intersection = ray.getIntersection2D(line_segment)) {
          invisible_edges.emplace_back(intersection.get(), ray.get2DVector(), diagonal_length);
        }
      }
    }
    // add rays through hull points
    for (const auto & point : hull) {
      auto ray = math::geometry::LineSegment(origin_.position, point);
      invisible_edges.emplace_back(point, ray.get2DVector(), diagonal_length);
    }
  }

  // fill invisible area
  {
    auto invisible_edge_cells = std::vector<std::pair<size_t, size_t>>();
    for (const auto & edge : invisible_edges) {
      fillByIntersection(edge, invisible_cost, invisible_edge_cells);
    }
    fillInside(invisible_edge_cells, invisible_cost);
  }

  // fill occupied area
  {
    auto occupied_edge_cells = std::vector<std::pair<size_t, size_t>>();
    for (const auto & edge : occupied_edges) {
      fillByIntersection(edge, occupied_cost, occupied_edge_cells);
    }
    fillInside(occupied_edge_cells, occupied_cost);
  }
}

const std::vector<int8_t> & Grid::getData() { return values_; }

std::vector<int8_t> Grid::calculate(const geometry_msgs::msg::Pose & origin, const std::vector<std::unique_ptr<primitives::Primitive>> & primitives) {
  namespace geom = boost::geometry;
  using point_t = geom::model::point<double, 2, geom::cs::cartesian>;
  using polygon_t = geom::model::polygon<point_t>;
  using multi_polygon_t = geom::model::multi_polygon<polygon_t>;
  using linestring_t = geom::model::linestring<point_t>;
  using multi_linestring_t = geom::model::multi_linestring<linestring_t>;

  auto rad_point_lt = [](auto p, auto q) {
    double p_rad = std::atan2(geom::get<1>(p), geom::get<0>(p));
    double q_rad = std::atan2(geom::get<1>(q), geom::get<0>(q));
    return p_rad - q_rad;
  };

  auto rad_line_lt = [](auto l, auto m) {
    return rad_point_lt(l[0], m[0]);
  };

  auto invisible_borders = multi_linestring_t();
  auto zero_line = linestring_t { { 0.0, 0.0 }, { width, 0.0 } };
  for (const auto &primitive : primitives) {
    auto hull = linestring_t();
    for (auto p : primitive->get2DConvexHull()) {
      hull.push_back({ p.x, p.y });
    }

    double min_rad = 0.0, max_rad = 2 * M_PI;
    point_t min_p, max_p;
    auto intersection = std::vector<point_t>();
    if (geom::intersection(hull, zero_line, intersection)) {
      for (auto p : hull) {
        auto rad = std::atan2(geom::get<1>(p), geom::get<0>(p));
        if (rad > M_PI) {
          if (rad < min_rad) {
            min_rad = rad;
            min_p = p;
          }
        } else {
          if (rad > max_rad) {
            max_rad = rad;
            max_p = p;
          }
        }
      }

      invisible_borders.push_back({ min_p, intersection[0] });
      invisible_borders.push_back({ intersection[0], max_p });
    } else {
      auto [ min_rad_p, max_rad_p ] = std::minmax_element(hull.begin(), hull.end(), rad_point_lt);
      invisible_borders.push_back({ *min_rad_p, *max_rad_p });
    }
  }

  std::sort(invisible_borders.begin(), invisible_borders.end(), rad_line_lt);

  // TODO: calculate polygon of invisible area
}

bool Grid::fillByRowCol(size_t row, size_t col, int8_t data)
{
  if (row >= width || col >= height) {
    return false;
  }
  values_[width * col + row] = data;
  return true;
}

void Grid::reset(const geometry_msgs::msg::Pose & origin)
{
  origin_ = origin;
  values_.assign(values_.size(), 0);
}

}  // namespace simple_sensor_simulator
