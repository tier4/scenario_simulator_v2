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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_CELL_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_CELL_HPP_

#include <array>
#include <boost/optional.hpp>
#include <geometry_math/polygon/line_segment.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <vector>

namespace simple_sensor_simulator
{
std::vector<geometry_msgs::msg::Point> getIntersection2D(
  const std::vector<geometry_math::LineSegment> & lines);

class GridCell
{
public:
  GridCell(
    const geometry_msgs::msg::Pose & origin, double size, size_t index, size_t row, size_t col);
  const geometry_msgs::msg::Pose origin;
  const double size;
  const size_t index;
  const size_t row;
  const size_t col;
  bool isIntersect2D(const geometry_math::LineSegment & line) const;
  bool isIntersect2D(const std::vector<geometry_math::LineSegment> & line_segments) const;
  bool contains(const geometry_msgs::msg::Point & p) const;
  bool contains(const std::vector<geometry_msgs::msg::Point> & points) const;
  int8_t getData() const;
  void setData(int8_t data);

private:
  int8_t data_;
  geometry_msgs::msg::Point transformToWorld(const geometry_msgs::msg::Point & point) const;
  std::array<geometry_math::LineSegment, 4> getLineSegments() const;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_CELL_HPP_
