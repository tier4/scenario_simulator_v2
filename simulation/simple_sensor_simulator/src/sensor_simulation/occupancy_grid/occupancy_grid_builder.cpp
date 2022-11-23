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
#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid_traversal.hpp>
#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/occupancy_grid_builder.hpp>

namespace simple_sensor_simulator
{
OccupancyGridBuilder::OccupancyGridBuilder(
  double resolution, size_t height, size_t width, int8_t occupied_cost, int8_t invisible_cost)
: resolution(resolution),
  height(height),
  width(width),

  occupied_cost(occupied_cost),
  invisible_cost(invisible_cost),

  occupied_grid_(height * width),
  invisible_grid_(height * width),
  values_(height * width),

  min_cols_(height),
  max_cols_(height)
{
}

auto OccupancyGridBuilder::transformToGrid(const PointType & p) const -> PointType
{
  using Quat = Eigen::Quaterniond;
  using Vec3 = Eigen::Vector3d;
  const auto & r = origin_.orientation;
  const auto & o = origin_.position;
  const auto np =
    (Quat(r.w, r.x, r.y, r.z).conjugate() * Vec3(p.x, p.y, p.z) - Vec3(o.x, o.y, o.z)).eval();
  return makePoint(np.x(), np.y(), np.z());
}

auto OccupancyGridBuilder::transformToPixel(const PointType & p) const -> PointType
{
  using Vec2 = Eigen::Vector2d;
  const auto np = ((Vec2(p.x, p.y) + Vec2(width, height) * resolution / 2) / resolution).eval();
  return makePoint(np.x(), np.y());
}

auto OccupancyGridBuilder::makePoint(double x, double y, double z) const -> PointType
{
  auto res = PointType();
  res.x = x, res.y = y, res.z = z;
  return res;
}

auto OccupancyGridBuilder::makeOccupiedArea(const PrimitiveType & primitive) const -> PolygonType
{
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Ring = bg::model::ring<Point>;

  auto primitive_ring = Ring();
  for (auto & e : primitive.get2DConvexHull()) {
    auto p = transformToGrid(e);
    primitive_ring.emplace_back(p.x, p.y);
  }

  const auto real_width = width * resolution / 2;
  const auto real_height = height * resolution / 2;

  auto grid_ring = Ring{
    {+real_width, +real_height},  // top right
    {+real_width, -real_height},  // bottom right
    {-real_width, -real_height},  // bottom left
    {-real_width, +real_height},  // top left
  };

  auto intersection_polygon = std::vector<Ring>();
  bg::intersection(primitive_ring, grid_ring, intersection_polygon);

  auto result = PolygonType();
  if (not intersection_polygon.empty()) {
    for (auto & p : intersection_polygon.front()) {
      result.emplace_back(makePoint(p.x(), p.y()));
    }
  }
  return result;
}

auto OccupancyGridBuilder::makeInvisibleArea(const PolygonType & occupied_polygon) const
  -> PolygonType
{
  if (occupied_polygon.empty()) {
    return {};
  }

  const auto real_width = width * resolution / 2;
  const auto real_height = height * resolution / 2;

  auto corners = PolygonType{
    makePoint(-real_width, -real_height),  // bottom left
    makePoint(+real_width, -real_height),  // bottom right
    makePoint(+real_width, +real_height),  // top right
    makePoint(-real_width, +real_height),  // top left
  };

  {
    bool overlap_origin = true;
    for (size_t i = 0; i < occupied_polygon.size(); ++i) {
      const auto & p = occupied_polygon[i];
      const auto & q = occupied_polygon[(i + 1) % occupied_polygon.size()];
      overlap_origin &= p.x * q.y - q.x * p.y < 0;
    }
    if (overlap_origin) return corners;
  }

  const auto projection = [&](const PointType & p, size_t i) -> PointType {
    switch (i % 4) {
      default:
        return makePoint(-real_width, p.y * -real_width / p.x);  // left
      case 1:
        return makePoint(p.x * -real_height / p.y, -real_height);  // bottom
      case 2:
        return makePoint(+real_width, p.y * +real_width / p.x);  // right
      case 3:
        return makePoint(p.x * +real_height / p.y, +real_height);  // top
    }
  };

  auto res = PolygonType();
  {
    auto angle = [](const PointType & p) { return std::atan2(p.y, p.x); };

    auto min_max_p = std::minmax_element(
      occupied_polygon.begin(), occupied_polygon.end(),
      [&](const PointType & p, const PointType & q) { return angle(p) < angle(q); });

    if (auto [min_p, max_p] = min_max_p; angle(*max_p) - angle(*min_p) > M_PI) {
      min_max_p = std::minmax_element(
        occupied_polygon.begin(), occupied_polygon.end(),
        [&](const PointType & p, const PointType & q) {
          auto adjust = [](double theta) { return theta < 0 ? theta + 2 * M_PI : theta; };
          return adjust(angle(p)) < adjust(angle(q));
        });
    }

    auto [min_p, max_p] = min_max_p;
    double min_ang = angle(*min_p);
    double max_ang = angle(*max_p);
    if (min_ang > max_ang) max_ang += 2 * M_PI;

    size_t i = 0;
    for (; angle(corners[i % 4]) + 2 * M_PI * (i / 4) <= min_ang; ++i) {
    }
    res.emplace_back(*min_p);
    res.emplace_back(projection(*min_p, i));

    for (; angle(corners[i % 4]) + 2 * M_PI * (i / 4) < max_ang; ++i) {
      res.emplace_back(corners[i % 4]);
    }
    res.emplace_back(projection(*max_p, i));
    res.emplace_back(*max_p);
  }
  return res;
}

auto OccupancyGridBuilder::addPolygon(MarkerGridType & grid, const PolygonType & convex_hull)
  -> void
{
  min_cols_.assign(min_cols_.size(), width);
  max_cols_.assign(max_cols_.size(), -1);

  for (size_t i = 0; i < convex_hull.size(); ++i) {
    const auto p = transformToPixel(convex_hull[i]);
    const auto q = transformToPixel(convex_hull[(i + 1) % convex_hull.size()]);
    for (auto [col, row] : GridTraversal(p.x, p.y, q.x, q.y)) {
      if (row >= 0 && row < int32_t(height)) {
        min_cols_[row] = std::min(min_cols_[row], col);
        max_cols_[row] = std::max(max_cols_[row], col);
      }
    }
  }

  for (size_t row = 0; row < height; ++row) {
    auto min_col = min_cols_[row];
    auto max_col = max_cols_[row] + 1;

    if (max_col <= 0 || min_col >= int32_t(width)) {
      continue;
    }
    if (min_col <= 0) {
      ++grid[width * row];
    } else {
      ++grid[width * row + min_col];
    }
    if (max_col < int32_t(width)) {
      --grid[width * row + max_col];
    }
  }
}

auto OccupancyGridBuilder::add(const PrimitiveType & primitive) -> void
{
  {
    constexpr auto count_max = std::numeric_limits<MarkerCounterType>::max();
    if (primitive_count_++ == count_max) {
      throw std::runtime_error(
        "Grid cannot hold more than " + std::to_string(count_max) + " primitives");
    }
  }

  auto occupied_area = makeOccupiedArea(primitive);

  auto invisible_area = makeInvisibleArea(occupied_area);

  // mark invisible area
  addPolygon(invisible_grid_, invisible_area);

  // mark occupied area
  addPolygon(occupied_grid_, occupied_area);
}

auto OccupancyGridBuilder::build() -> void
{
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

auto OccupancyGridBuilder::get() const -> const OccupancyGridType & { return values_; }

auto OccupancyGridBuilder::reset(const PoseType & origin) -> void
{
  origin_ = origin;
  primitive_count_ = 0;
  invisible_grid_.assign(invisible_grid_.size(), 0);
  occupied_grid_.assign(occupied_grid_.size(), 0);
}

}  // namespace simple_sensor_simulator
