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
  LineSegment & operator=(const LineSegment &);
  const geometry_msgs::msg::Point start_point;
  const geometry_msgs::msg::Point end_point;
  bool isIntersect2D(const LineSegment & l0) const;
  boost::optional<geometry_msgs::msg::Point> getIntersection2D(const LineSegment & line) const;
  boost::optional<geometry_msgs::msg::Point> getIntersection2DWithXAxis(double x) const;

  geometry_msgs::msg::Vector3 getVector() const;
  geometry_msgs::msg::Vector3 get2DVector() const;
  double getLength() const;
  double get2DLength() const;
  double getSlope() const;
  double getIntercept() const;
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
  int8_t getData() const;
  void setData(int8_t data);

private:
  int8_t data_;
  geometry_msgs::msg::Point transformToWorld(const geometry_msgs::msg::Point & point) const;
  std::array<LineSegment, 4> getLineSegments() const;
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__OCCUPANCY_GRID__GRID_CELL_HPP_
