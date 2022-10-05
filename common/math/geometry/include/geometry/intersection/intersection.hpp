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

#ifndef GEOMETRY__INTERSECTION__INTERSECTION_HPP_
#define GEOMETRY__INTERSECTION__INTERSECTION_HPP_

#include <optional>
#include <geometry/polygon/line_segment.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace math
{
namespace geometry
{
bool isIntersect2D(const LineSegment & line0, const LineSegment & line1);
bool isIntersect2D(const std::vector<LineSegment> & lines);
std::optional<geometry_msgs::msg::Point> getIntersection2D(
  const LineSegment & line0, const LineSegment & line1);
std::vector<geometry_msgs::msg::Point> getIntersection2D(const std::vector<LineSegment> & lines);
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__INTERSECTION__INTERSECTION_HPP_
