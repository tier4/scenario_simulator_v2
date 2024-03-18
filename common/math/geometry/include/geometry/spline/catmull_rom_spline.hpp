// Copyright 2015 TIER IV.inc. All rights reserved.
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
#ifndef GEOMETRY__SPLINE__CATMULL_ROM_SPLINE_HPP_
#define GEOMETRY__SPLINE__CATMULL_ROM_SPLINE_HPP_

#include <exception>
#include <geometry/polygon/line_segment.hpp>
#include <geometry/spline/catmull_rom_spline_interface.hpp>
#include <geometry/spline/hermite_curve.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace math
{
namespace geometry
{
class CatmullRomSpline : public CatmullRomSplineInterface
{
public:
  CatmullRomSpline() = default;
  explicit CatmullRomSpline(const std::vector<geometry_msgs::msg::Point> & control_points);
  auto getLength() const -> double override { return total_length_; }
  auto getMaximum2DCurvature() const -> double;
  auto getPoint(const double s) const -> geometry_msgs::msg::Point;
  auto getPoint(const double s, const double offset) const -> geometry_msgs::msg::Point;
  auto getTangentVector(const double s) const -> geometry_msgs::msg::Vector3;
  auto getNormalVector(const double s) const -> geometry_msgs::msg::Vector3;
  auto getPose(const double s, const bool fill_pitch = true) const -> geometry_msgs::msg::Pose;
  auto getTrajectory(
    const double start_s, const double end_s, const double resolution,
    const double offset = 0.0) const -> std::vector<geometry_msgs::msg::Point>;
  auto getSValue(const geometry_msgs::msg::Pose & pose, double threshold_distance = 3.0) const
    -> std::optional<double>;
  auto getSquaredDistanceIn2D(const geometry_msgs::msg::Point & point, const double s) const
    -> double;
  auto getSquaredDistanceVector(const geometry_msgs::msg::Point & point, const double s) const
    -> geometry_msgs::msg::Vector3;
  auto getCollisionPointsIn2D(
    const std::vector<geometry_msgs::msg::Point> & polygon,
    const bool search_backward = false) const -> std::set<double>;
  auto getCollisionPointIn2D(
    const geometry_msgs::msg::Point & point0, const geometry_msgs::msg::Point & point1,
    const bool search_backward = false) const -> std::optional<double>;
  auto getCollisionPointIn2D(
    const std::vector<geometry_msgs::msg::Point> & polygon,
    const bool search_backward = false) const -> std::optional<double> override;
  auto getPolygon(const double width, const size_t num_points = 30, const double z_offset = 0)
    -> std::vector<geometry_msgs::msg::Point>;
  const std::vector<geometry_msgs::msg::Point> control_points;

private:
  auto getRightBounds(const double width, const size_t num_points = 30, const double z_offset = 0)
    const -> std::vector<geometry_msgs::msg::Point>;
  auto getLeftBounds(const double width, const size_t num_points = 30, const double z_offset = 0)
    const -> std::vector<geometry_msgs::msg::Point>;
  auto getSInSplineCurve(const size_t curve_index, const double s) const -> double;
  auto getCurveIndexAndS(const double s) const -> std::pair<size_t, double>;
  auto checkConnection() const -> bool;
  auto equals(const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1) const
    -> bool;
  std::vector<LineSegment> line_segments_;
  std::vector<HermiteCurve> curves_;
  std::vector<double> length_list_;
  std::vector<double> maximum_2d_curvatures_;
  double total_length_;
};
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__SPLINE__CATMULL_ROM_SPLINE_HPP_
