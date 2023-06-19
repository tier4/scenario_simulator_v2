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
  double getLength() const override { return total_length_; }
  double getMaximum2DCurvature() const;
  const geometry_msgs::msg::Point getPoint(double s) const;
  const geometry_msgs::msg::Point getPoint(double s, double offset) const;
  const geometry_msgs::msg::Vector3 getTangentVector(double s) const;
  const geometry_msgs::msg::Vector3 getNormalVector(double s) const;
  const geometry_msgs::msg::Pose getPose(double s) const;
  const std::vector<geometry_msgs::msg::Point> getTrajectory(
    const double start_s, const double end_s, const double resolution,
    const double offset = 0.0) const;
  std::optional<double> getSValue(
    const geometry_msgs::msg::Pose & pose, double threshold_distance = 3.0) const;
  double getSquaredDistanceIn2D(const geometry_msgs::msg::Point & point, double s) const;
  geometry_msgs::msg::Vector3 getSquaredDistanceVector(
    const geometry_msgs::msg::Point & point, double s) const;
  std::optional<double> getCollisionPointIn2D(
    const geometry_msgs::msg::Point & point0, const geometry_msgs::msg::Point & point1,
    const bool search_backward = false) const;
  std::optional<double> getCollisionPointIn2D(
    const std::vector<geometry_msgs::msg::Point> & polygon,
    const bool search_backward = false) const override;
  const std::vector<geometry_msgs::msg::Point> getPolygon(
    const double width, const size_t num_points = 30, const double z_offset = 0);
  const std::vector<geometry_msgs::msg::Point> control_points;

private:
  const std::vector<geometry_msgs::msg::Point> getRightBounds(
    const double width, const size_t num_points = 30, const double z_offset = 0) const;
  const std::vector<geometry_msgs::msg::Point> getLeftBounds(
    const double width, const size_t num_points = 30, const double z_offset = 0) const;
  double getSInSplineCurve(const size_t curve_index, const double s) const;
  std::pair<size_t, double> getCurveIndexAndS(const double s) const;
  bool checkConnection() const;
  bool equals(geometry_msgs::msg::Point p0, geometry_msgs::msg::Point p1) const;
  std::vector<LineSegment> line_segments_;
  std::vector<HermiteCurve> curves_;
  std::vector<double> length_list_;
  std::vector<double> maximum_2d_curvatures_;
  double total_length_;
};
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__SPLINE__CATMULL_ROM_SPLINE_HPP_
