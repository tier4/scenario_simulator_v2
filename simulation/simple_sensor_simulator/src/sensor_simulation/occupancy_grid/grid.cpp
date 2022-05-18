// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <simple_sensor_simulator/sensor_simulation/occupancy_grid/grid.hpp>

namespace simple_sensor_simulator
{
LineSegment::LineSegment(
  const geometry_msgs::msg::Point & start_point, const geometry_msgs::msg::Point & end_point)
: start_point(start_point), end_point(end_point)
{
}

LineSegment::~LineSegment() {}

GridCell::GridCell(const geometry_msgs::msg::Point & origin, double size, size_t index)
: origin(origin), size(size), index(index)
{
}

Grid::Grid(double resolution, double height, double width)
: resolution(resolution), height(height), width(width)
{
}

std::vector<GridCell> Grid::getCell(
  const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive,
  const geometry_msgs::msg::Pose & sensor_pose) const
{
  std::vector<GridCell> ret;
  const auto x_max = primitive->getMax(simple_sensor_simulator::Axis::X);
  const auto y_max = primitive->getMax(simple_sensor_simulator::Axis::Y);
  const auto x_min = primitive->getMin(simple_sensor_simulator::Axis::X);
  const auto y_min = primitive->getMin(simple_sensor_simulator::Axis::Y);
  for (size_t h = 0; h < height; h++) {
    for (size_t w = 0; w < width; w++) {
      if (x_max && y_max && x_min && y_min) {
        geometry_msgs::msg::Point origin;
        origin.x = w - 0.5 * width;
        origin.y = h - 0.5 * height;
        double x_min_cell = (origin.x - 0.5) * resolution;
        double y_min_cell = (origin.y - 0.5) * resolution;
        double x_max_cell = (origin.x + 0.5) * resolution;
        double y_max_cell = (origin.y + 0.5) * resolution;
        if (
          x_min_cell <= x_max && x_max <= x_max_cell && y_min_cell <= y_max &&
          y_max <= y_max_cell) {
          ret.emplace_back(GridCell(origin, resolution, width * h + w));
        }
      }
    }
  }
  return ret;
}
}  // namespace simple_sensor_simulator
