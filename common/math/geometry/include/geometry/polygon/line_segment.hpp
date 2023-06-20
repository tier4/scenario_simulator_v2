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

#ifndef GEOMETRY__POLYGON__LINE_SEGMENT_
#define GEOMETRY__POLYGON__LINE_SEGMENT_

#include <geometry/polygon/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <optional>

namespace math
{
namespace geometry
{
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
  bool isIntersect2D(const geometry_msgs::msg::Point & point) const;
  bool isIntersect2D(const LineSegment & l0) const;
  std::optional<double> getIntersection2DSValue(const geometry_msgs::msg::Point & point) const;
  std::optional<double> getIntersection2DSValue(const LineSegment & line) const;
  std::optional<geometry_msgs::msg::Point> getIntersection2D(const LineSegment & line) const;
  std::optional<geometry_msgs::msg::Point> getIntersection2DWithXAxis(double x) const;
  geometry_msgs::msg::Vector3 getVector() const;
  geometry_msgs::msg::Vector3 get2DVector() const;
  double getLength() const;
  double get2DLength() const;
  double getSlope() const;
};

std::vector<LineSegment> getLineSegments(const std::vector<geometry_msgs::msg::Point> & points);
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__POLYGON__LINE_SEGMENT_
