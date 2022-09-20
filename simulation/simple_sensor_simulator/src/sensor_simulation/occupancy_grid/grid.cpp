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

  occupied_grid_(height * width),
  invisible_grid_(height * width),
  values_(height * width),

  mincols_(height),
  maxcols_(height)
{
}

auto Grid::constructPoint(double x, double y, double z = 0) const -> geometry_msgs::msg::Point
{
  auto p = geometry_msgs::msg::Point();
  p.x = x, p.y = y, p.z = z;
  return p;
}

auto Grid::transformToGrid(const geometry_msgs::msg::Point & world_point) const
  -> geometry_msgs::msg::Point
{
  namespace quat_op = quaternion_operation;
  auto rot = quat_op::getRotationMatrix(quat_op::conjugate(origin_.orientation));
  auto p = Eigen::Vector3d(world_point.x, world_point.y, world_point.z);
  auto q = Eigen::Vector3d(origin_.position.x, origin_.position.y, origin_.position.z);
  p = rot * p - q;
  return constructPoint(p(0), p(1), p(2));
}

auto Grid::transformToPixel(const geometry_msgs::msg::Point & grid_point) const
  -> geometry_msgs::msg::Point
{
  return constructPoint(
    (grid_point.x + height * resolution * 0.5) / resolution,
    (grid_point.y + width * resolution * 0.5) / resolution);
}

auto Grid::minmaxAnglePoint(const std::vector<geometry_msgs::msg::Point> & polygon) const
{
  using geometry_msgs::msg::Point;

  auto res = std::minmax_element(
    polygon.begin(), polygon.end(),
    [&](const Point & p, const Point & q) { return std::atan2(p.y, p.x) < std::atan2(q.y, q.x); });

  auto [minp, maxp] = res;
  if (std::atan2(maxp->y, maxp->x) - std::atan2(minp->y, minp->x) > M_PI) {
    res =
      std::minmax_element(polygon.begin(), polygon.end(), [&](const Point & p, const Point & q) {
        auto prad = std::atan2(p.y, p.x);
        auto qrad = std::atan2(q.y, q.x);

        if (prad < 0) prad += 2 * M_PI;
        if (qrad < 0) qrad += 2 * M_PI;

        return prad < qrad;
      });
  }

  return res;
}

auto Grid::constructOccupiedConvexHull(const primitives::Primitive & primitive) const
  -> std::vector<geometry_msgs::msg::Point>
{
  auto res = primitive.get2DConvexHull();
  for (auto & p : res) {
    p = transformToGrid(p);
  }
  return res;
}

auto Grid::constructInvisibleConvexHull(
  const std::vector<geometry_msgs::msg::Point> & occupied_convex_hull) const
  -> std::vector<geometry_msgs::msg::Point>
{
  using geometry_msgs::msg::Point;

  const auto realw = width * resolution / 2;
  const auto realh = height * resolution / 2;

  const auto corners = [&](size_t i) {
    switch (i % 4) {
      default:
        return constructPoint(-realw, -realh);  // bottom left
      case 1:
        return constructPoint(realw, -realh);  // bottom right
      case 2:
        return constructPoint(realw, realh);  // top right
      case 3:
        return constructPoint(-realw, realh);  // top left
    }
  };

  const auto projection = [&](const Point & p, size_t i) {
    switch (i % 4) {
      default:
        return constructPoint(-realw, p.y * -realw / p.x);  // left
      case 1:
        return constructPoint(p.x * -realh / p.y, -realh);  // bottom
      case 2:
        return constructPoint(realw, p.y * realw / p.x);  // right
      case 3:
        return constructPoint(p.x * realh / p.y, realh);  // top
    }
  };

  auto res = std::vector<Point>();
  {
    auto [minp, maxp] = minmaxAnglePoint(occupied_convex_hull);
    double minang = std::atan2(minp->y, minp->x);
    double maxang = std::atan2(maxp->y, maxp->x);
    if (minang > maxang) maxang += 2 * M_PI;

    size_t i = 0;
    for (;; ++i) {
      auto corner = corners(i);
      if (std::atan2(corner.y, corner.x) >= minang) break;
    }

    res.emplace_back(*minp);
    res.emplace_back(projection(*minp, i));

    for (;; ++i) {
      auto corner = corners(i);
      if (std::atan2(corner.y, corner.x) + 2 * M_PI * (i / 4) >= maxang) break;
      res.emplace_back(corner);
    }

    res.emplace_back(projection(*maxp, i));
    res.emplace_back(*maxp);
  }
  return res;
}

auto Grid::markConvexHull(
  std::vector<int8_t> & grid, const std::vector<geometry_msgs::msg::Point> & convex_hull) -> void
{
  mincols_.assign(mincols_.size(), width);
  maxcols_.assign(maxcols_.size(), -1);

  for (size_t i = 0; i < convex_hull.size(); ++i) {
    const auto p = transformToPixel(convex_hull[i]);
    const auto q = transformToPixel(convex_hull[(i + 1) % convex_hull.size()]);
    traverse(p, q, [&](ssize_t col, ssize_t row) {
      if (row >= 0 && row < ssize_t(height)) {
        mincols_[row] = std::min(mincols_[row], col);
        maxcols_[row] = std::max(maxcols_[row], col);
      }
    });
  }

  for (size_t row = 0; row < height; ++row) {
    if (auto col = mincols_[row]; col >= 0 && col < ssize_t(width)) {
      grid[width * row + col] += 1;
    }
    if (auto col = maxcols_[row]; col >= 0 && col + 1 < ssize_t(width)) {
      grid[width * row + col + 1] -= 1;
    }
  }
}

auto Grid::add(const primitives::Primitive & primitive) -> void
{
  auto occupied_convex_hull = constructOccupiedConvexHull(primitive);
  auto invisible_convex_hull = constructInvisibleConvexHull(occupied_convex_hull);

  // mark invisible area
  markConvexHull(invisible_grid_, invisible_convex_hull);

  // mark occupied area
  markConvexHull(occupied_grid_, occupied_convex_hull);
}

auto Grid::construct() -> void
{
  // Imos Method
  // https://imoz.jp/algorithms/imos_method.html (Japanese)

  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col + 1 < width; ++col) {
      invisible_grid_[row * width + col + 1] += invisible_grid_[row * width + col];
    }
  }

  for (size_t row = 0; row < height; ++row) {
    for (size_t col = 0; col + 1 < width; ++col) {
      occupied_grid_[row * width + col + 1] += occupied_grid_[row * width + col];
    }
  }

  for (size_t i = 0; i < height * width; ++i) {
    values_[i] = occupied_grid_[i] ? occupied_cost : invisible_grid_[i] ? invisible_cost : 0;
  }
}

auto Grid::get() const -> const std::vector<int8_t> & { return values_; }

auto Grid::reset(const geometry_msgs::msg::Pose & origin) -> void
{
  origin_ = origin;
  invisible_grid_.assign(invisible_grid_.size(), 0);
  occupied_grid_.assign(occupied_grid_.size(), 0);
}

}  // namespace simple_sensor_simulator
