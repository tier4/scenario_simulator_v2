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
  auto grouped = std::vector<std::vector<size_t>>(height);
  for (auto & [ p, q ] : segments) {
    traverse(transformToPixel(p), transformToPixel(q), [&](ssize_t col, ssize_t row) {
      if (col >= 0 && col < width && row >= 0 && row < height) {
        grouped.at(row).emplace_back(col);
      }
    });
  }

  for (size_t row = 0; row < height; ++row) {
    if (const auto & cols = grouped[row]; cols.size() > 1) {
      auto [mincol, maxcol] = std::minmax_element(cols.begin(), cols.end());
      for (auto col = *mincol; col <= *maxcol; ++col) {
        values_.at(width * row + col) = data;
      }
    }
  }
}

void Grid::addPrimitive(const std::unique_ptr<primitives::Primitive> & primitive)
{
  using geometry_msgs::msg::Point;
  using math::geometry::LineSegment;

  auto make_point = [](double x, double y) {
    auto p = Point();
    p.x = x, p.y = y;
    return p;
  };

  auto hull = primitive->get2DConvexHull();
  for (auto &p : hull) {
    p = transformToGrid(p);
  }

  auto invisible_edges = std::vector<LineSegment>();
  {
    auto minp = decltype(hull)::iterator();
    auto maxp = decltype(hull)::iterator();

    {
      auto ord = [&](const Point & p, const Point & q) {
        return std::atan2(p.y, p.x) < std::atan2(q.y, q.x);
      };
      std::tie(minp, maxp) = std::minmax_element(hull.begin(), hull.end(), ord);
    }

    auto minang = std::atan2(minp->y, minp->x);
    auto maxang = std::atan2(maxp->y, maxp->x);

    if (maxang - minang > M_PI) {
      auto ord = [&](const Point & p, const Point & q) {
        auto prad = std::atan2(p.y, p.x);
        auto qrad = std::atan2(q.y, q.x);

        if (prad < 0) prad += 2 * M_PI;
        if (qrad < 0) qrad += 2 * M_PI;

        return prad < qrad;
      };
      std::tie(minp, maxp) = std::minmax_element(hull.begin(), hull.end(), ord);
      minang = std::atan2(minp->y, minp->x);
      maxang = std::atan2(maxp->y, maxp->x) + 2 * M_PI;
    }

    {
      auto realw = width * resolution / 2;
      auto realh = height * resolution / 2;

      auto corners = std::vector<Point> {
        make_point(-realw, -realh), // bottom left
        make_point(realw, -realh), // bottom right
        make_point(realw, realh), // top right
        make_point(-realw, realh), // top left
      };

      auto projection = [&](const Point & p, size_t wall) {
        switch (wall) {
          case 0: return make_point(-realw, p.y * -realw / p.x); // left
          case 1: return make_point(p.x * -realh / p.y, -realh); // bottom
          case 2: return make_point(realw, p.y * realw / p.x); // right
          case 3: return make_point(p.x * realh / p.y, realh); // top
        }
      };

      invisible_edges.emplace_back(*minp, *maxp);

      auto i = size_t(0);
      auto last = Point();
      for (;; ++i) {
        auto & corner = corners[i];
        if (std::atan2(corner.y, corner.x) >= minang) {
          auto p = projection(*minp, i);
          invisible_edges.emplace_back(*minp, p);
          last = p;
          break;
        }
      }

      for (;; ++i) {
        auto & corner = corners[i % 4];
        if (std::atan2(corner.y, corner.x) >= maxang) {
          auto p = projection(*maxp, i % 4);
          invisible_edges.emplace_back(last, p);
          invisible_edges.emplace_back(p, *maxp);
          break;
        }

        invisible_edges.emplace_back(last, corner);
        last = corner;
      }
    }
  }

  // fill invisible area
  fillInside(invisible_edges, invisible_cost);

  // fill occupied area
  fillInside(math::geometry::getLineSegments(hull), occupied_cost);
}

const std::vector<int8_t> & Grid::getData() { return values_; }

void Grid::reset(const geometry_msgs::msg::Pose & origin)
{
  origin_ = origin;
  values_.assign(values_.size(), 0);
}

}  // namespace simple_sensor_simulator
