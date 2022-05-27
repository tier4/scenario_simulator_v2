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
#include <boost/optional.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <vector>

namespace simple_sensor_simulator
{
std::vector<geometry_msgs::msg::Point> get2DConvexHull(
  const std::vector<geometry_msgs::msg::Point> & points);

class LineSegment
{
public:
  LineSegment(
    const geometry_msgs::msg::Point & start_point, const geometry_msgs::msg::Point & end_point);
  LineSegment(
    const geometry_msgs::msg::Point & start_point, const geometry_msgs::msg::Vector3 & vec,
    double length);
  ~LineSegment();
  const geometry_msgs::msg::Point start_point;
  const geometry_msgs::msg::Point end_point;
  bool isIntersect2D(const LineSegment & l0) const;
  boost::optional<geometry_msgs::msg::Point> getIntersection2D(const LineSegment & line) const;
  geometry_msgs::msg::Vector3 getVector() const;
  double getLength() const;
};

std::vector<LineSegment> getLineSegments(const std::vector<geometry_msgs::msg::Point> & points);
std::vector<geometry_msgs::msg::Point> getIntersection2D(const std::vector<LineSegment> & lines);

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
  bool isIntersect2D(const LineSegment & line) const;
  bool isIntersect2D(const std::vector<LineSegment> & line_segments) const;
  bool contains(const geometry_msgs::msg::Point & p) const;
  bool contains(const std::vector<geometry_msgs::msg::Point> & points) const;
  bool operator==(const GridCell & rhs) const;
  bool operator!=(const GridCell & rhs) const;
  bool operator<(const GridCell & rhs) const;
  bool operator>(const GridCell & rhs) const;
  bool operator<=(const GridCell & rhs) const;
  bool operator>=(const GridCell & rhs) const;
  GridCell & operator=(const GridCell & rhs);

private:
  geometry_msgs::msg::Point transformToWorld(const geometry_msgs::msg::Point & point) const;
  std::array<LineSegment, 4> getLineSegments() const;
};

class Grid
{
public:
  Grid(const geometry_msgs::msg::Pose & origin, double resolution, double height, double width);
  const double resolution;
  const double height;
  const double width;
  const geometry_msgs::msg::Pose origin;
  std::vector<GridCell> getInvisibleCell(
    const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive) const;
  std::vector<GridCell> getOccupiedCell(
    const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive) const;

private:
  std::vector<GridCell> getAllCells() const;
  std::vector<GridCell> getOccupiedCandidates(
    const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive) const;
  std::vector<GridCell> filterByRow(const std::vector<GridCell> & cells, size_t row) const;
  std::vector<GridCell> filterByCol(const std::vector<GridCell> & cells, size_t col) const;
  std::vector<GridCell> filterByIndex(
    const std::vector<GridCell> & cells, std::vector<size_t> index) const;
  std::vector<GridCell> filterByIntersection(
    const std::vector<GridCell> & cells, const std::vector<LineSegment> & line_segments) const;
  std::vector<GridCell> filterByContain(
    const std::vector<GridCell> & cells,
    const std::vector<geometry_msgs::msg::Point> & points) const;
  std::vector<size_t> getRows(const std::vector<GridCell> & cells) const;
  std::vector<size_t> getCols(const std::vector<GridCell> & cells) const;
  std::vector<GridCell> merge(
    const std::vector<GridCell> & cells0, const std::vector<GridCell> & cells1) const;
  std::vector<size_t> getFillIndex(const std::vector<GridCell> & cells) const;
  std::array<LineSegment, 4> getOutsideLineSegments() const;
  std::vector<geometry_msgs::msg::Point> raycastToOutside(
    const std::unique_ptr<simple_sensor_simulator::primitives::Primitive> & primitive) const;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_HPP_