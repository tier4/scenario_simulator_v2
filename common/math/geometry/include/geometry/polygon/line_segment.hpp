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
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <optional>

namespace math
{
namespace geometry
{
class LineSegment
{
public:
  const geometry_msgs::msg::Point start_point;
  const geometry_msgs::msg::Point end_point;
  const geometry_msgs::msg::Vector3 vector;
  const geometry_msgs::msg::Vector3 vector_2d;
  const double length;
  const double length_2d;

  LineSegment(
    const geometry_msgs::msg::Point & start_point, const geometry_msgs::msg::Point & end_point);
  LineSegment(
    const geometry_msgs::msg::Point & start_point, const geometry_msgs::msg::Vector3 & vec,
    double length);
  ~LineSegment();
  LineSegment & operator=(const LineSegment &);

  auto isIntersect2D(const geometry_msgs::msg::Point & point) const -> bool;
  auto isIntersect2D(const LineSegment & line) const -> bool;
  auto isInBounds2D(const geometry_msgs::msg::Point & point) const -> bool;

  auto getPoint(const double s, const bool denormalize_s = false) const
    -> geometry_msgs::msg::Point;
  auto getPose(const double s, const bool denormalize_s = false, const bool fill_pitch = true) const
    -> geometry_msgs::msg::Pose;
  auto get2DIntersectionSValue(
    const geometry_msgs::msg::Point & point, const bool denormalize_s = false) const
    -> std::optional<double>;
  auto get2DIntersectionSValue(const LineSegment & line, const bool denormalize_s = false) const
    -> std::optional<double>;
  auto getIntersection2D(const LineSegment & line) const
    -> std::optional<geometry_msgs::msg::Point>;
  auto getSValue(
    const geometry_msgs::msg::Pose & pose, double threshold_distance, bool denormalize_s) const
    -> std::optional<double>;
  auto getNormalVector() const -> geometry_msgs::msg::Vector3;
  auto get2DVectorSlope() const -> double;
  auto getSquaredDistanceIn2D(
    const geometry_msgs::msg::Point & point, const double s, const bool denormalize_s = false) const
    -> double;
  auto getSquaredDistanceVector(
    const geometry_msgs::msg::Point & point, const double s, const bool denormalize_s = false) const
    -> geometry_msgs::msg::Vector3;
  auto relativePointPosition2D(const geometry_msgs::msg::Point & point) const -> int;

private:
  auto denormalize(const std::optional<double> & s, const bool throw_error_on_out_of_range = true)
    const -> std::optional<double>;
  auto denormalize(const double s) const -> double;
};

auto getLineSegments(
  const std::vector<geometry_msgs::msg::Point> & points, const bool close_start_end = false)
  -> std::vector<LineSegment>;
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__POLYGON__LINE_SEGMENT_
