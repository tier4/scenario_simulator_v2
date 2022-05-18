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
  GridCell(const geometry_msgs::msg::Point & origin, double size, size_t index);
  const geometry_msgs::msg::Point origin;
  const double size;
  const size_t index;
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
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_