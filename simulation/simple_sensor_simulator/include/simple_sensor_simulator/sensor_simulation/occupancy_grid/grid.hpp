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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_

#include <array>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <vector>

namespace simple_sensor_simulator
{
class LineSegment
{
public:
  LineSegment(
    const geometry_msgs::msg::Point & start_point, const geometry_msgs::msg::Point & end_point);
  ~LineSegment();
  const geometry_msgs::msg::Point start_point;
  const geometry_msgs::msg::Point end_point;
};

class GridCell
{
public:
  GridCell(
    const geometry_msgs::msg::Pose & origin, double size, size_t index, size_t row, size_t col);
  std::array<LineSegment, 4> getLineSegments() const;
  const geometry_msgs::msg::Pose origin;
  const double size;
  const size_t index;
  const size_t row;
  const size_t col;

private:
  geometry_msgs::msg::Point transformToWorld(const geometry_msgs::msg::Point & point) const;
};

class Grid
{
public:
  Grid(double resolution, double height, double width);
  const double resolution;
  const double height;
  const double width;
  std::vector<GridCell> getCell(
    const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive,
    const geometry_msgs::msg::Pose & sensor_pose) const;

private:
  std::vector<GridCell> getOccupiedCandidates(
    const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive,
    const geometry_msgs::msg::Pose & sensor_pose) const;
  std::vector<GridCell> filterByRow(const std::vector<GridCell> & cells, size_t row) const;
  std::vector<GridCell> filterByCol(const std::vector<GridCell> & cells, size_t col) const;
  std::vector<size_t> getRows(const std::vector<GridCell> & cells);
  std::vector<size_t> getCols(const std::vector<GridCell> & cells);
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_