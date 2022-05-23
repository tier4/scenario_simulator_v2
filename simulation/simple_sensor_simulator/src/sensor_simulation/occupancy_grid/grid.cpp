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

#include <rclcpp/rclcpp.hpp>
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
  if (!x_max || !y_max || !x_min || !y_min) {
    return ret;
  }
  int x_min_index =
    std::floor((x_min.get() - sensor_pose.position.x + resolution * 0.5 * height) / resolution);
  int x_max_index =
    std::ceil((x_max.get() - sensor_pose.position.x + resolution * 0.5 * width) / resolution);
  int y_min_index =
    std::floor((y_min.get() - sensor_pose.position.y + resolution * 0.5 * height) / resolution);
  int y_max_index =
    std::ceil((y_max.get() - sensor_pose.position.y + resolution * 0.5 * width) / resolution);
  for (int x_index = x_min_index; x_index <= x_max_index; x_index++) {
    for (int y_index = y_min_index; y_index <= y_max_index; y_index++) {
      geometry_msgs::msg::Point cell_origin;
      cell_origin.x = sensor_pose.position.x + (x_index - 0.5 * height) * resolution;
      cell_origin.y = sensor_pose.position.y + (y_index - 0.5 * width) * resolution;
      cell_origin.z = sensor_pose.position.z;
      ret.emplace_back(GridCell(cell_origin, resolution, width * y_index + x_index));
    }
  }
  return ret;
}
}  // namespace simple_sensor_simulator
