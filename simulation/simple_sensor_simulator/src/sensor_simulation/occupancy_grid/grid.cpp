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
      double half_rw = static_cast<double>(width) * resolution / 2;
      double half_rh = static_cast<double>(height) * resolution / 2;

      // enumerate corner coordinates
      corners[0].x = half_rw, corners[0].y = half_rh;
      corners[1].x = -half_rw, corners[1].y = half_rh;
      corners[2].x = half_rw, corners[2].y = -half_rh;
      corners[3].x = -half_rw, corners[3].y = -half_rh;

      // transform to world coordinate system
      for (auto & corner : corners) {
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

// this function assume that primitives do not overlap each other
std::vector<int8_t> Grid::calculate(const geometry_msgs::msg::Pose & origin, const std::vector<std::unique_ptr<primitives::Primitive>> & primitives) {
  using LineSegment = math::geometry::LineSegment;
  using Point = geometry_msgs::msg::Point;

  struct sweep_line_event {
    enum event_type { LineBegin, LineEnd } type;
    double angle;
    ssize_t index;

    bool operator<(const sweep_line_event &that) { return this->angle < that.angle; }
  };

  auto edges = std::vector<LineSegment>();
  for (auto & primitive : primitives) {
    auto hull = primitive->get2DConvexHull();
    for (size_t i = 0; i + 1 < hull.size(); ++i) {
      auto p = transformToGrid(hull[i]);
      auto q = transformToGrid(hull[(i + 1) % hull.size()]);

      auto p_rad = std::atan2(p.y, p.x);
      auto q_rad = std::atan2(q.y, q.x);

      if ((std::abs(q_rad - p_rad) > M_PI) ^ (p_rad < q_rad)) {
        edges.emplace_back(p, q);
      } else {
        edges.emplace_back(q, p);
      }
    }
  }

  auto edge_nearer = [&](size_t i, size_t j) {
    const auto &[ a, b ] = edges[i];
    const auto &[ p, q ] = edges[j];

    auto left_side = [](const Point &a, const Point &b, const Point &p) {
      return ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)) > 0;
    };
    return left_side(a, b, p) && left_side(a, b, q);
  };

  auto events = std::vector<sweep_line_event>();
  auto edge_indices = std::set<size_t, decltype(edge_nearer)>(edge_nearer);
  for (ssize_t i = 0; i < edges.size(); ++i) {
    const auto &[ p, q ] = edges[i];

    auto p_rad = std::atan2(p.y, p.x);
    auto q_rad = std::atan2(q.y, q.x);

    if (p_rad - q_rad > M_PI) {
      edge_indices.emplace(i);
    }

    events.push_back({ sweep_line_event::LineBegin, p_rad, i });
    events.push_back({ sweep_line_event::LineEnd, q_rad, i });
  }

  std::sort(events.begin(), events.end());

  // TODO: implement filling procedure
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
