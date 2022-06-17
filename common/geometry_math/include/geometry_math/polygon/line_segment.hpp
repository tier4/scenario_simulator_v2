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

#ifndef GEOMETRY_MATH__POLYGON__LINE_SEGMENT_
#define GEOMETRY_MATH__POLYGON__LINE_SEGMENT_

#include <boost/optional.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace geometry_math
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
}  // namespace geometry_math

#endif  // GEOMETRY_MATH__POLYGON__LINE_SEGMENT_
