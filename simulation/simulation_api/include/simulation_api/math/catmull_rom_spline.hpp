// Copyright 2015-2020 TierIV.inc. All rights reserved.
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
#ifndef SIMULATION_API__MATH__CATMULL_ROM_SPLINE_HPP_
#define SIMULATION_API__MATH__CATMULL_ROM_SPLINE_HPP_

#include <simulation_api/math/hermite_curve.hpp>

#include <openscenario_msgs/msg/catmull_rom_spline.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <exception>
#include <string>
#include <utility>

namespace simulation_api
{
namespace math
{
class SplineInterpolationError : public std::runtime_error
{
public:
  explicit SplineInterpolationError(const char * message)
  : runtime_error(message) {}
  explicit SplineInterpolationError(std::string message)
  : runtime_error(message.c_str()) {}
};

class CatmullRomSpline
{
public:
  explicit CatmullRomSpline(openscenario_msgs::msg::CatmullRomSpline spline);
  explicit CatmullRomSpline(std::vector<openscenario_msgs::msg::HermiteCurve> hermite_curves);
  explicit CatmullRomSpline(std::vector<geometry_msgs::msg::Point> control_points);
  double getLength() const {return total_length_;}
  double getMaximum2DCurventure() const;
  const geometry_msgs::msg::Point getPoint(double s) const;
  const geometry_msgs::msg::Vector3 getTangentVector(double s) const;
  const geometry_msgs::msg::Vector3 getNormalVector(double s) const;
  const geometry_msgs::msg::Pose getPose(double s) const;
  const std::vector<geometry_msgs::msg::Point> getTrajectory(int num_points) const;
  const std::vector<geometry_msgs::msg::Point> getTrajectory(
    double start_s, double end_s,
    double resolution) const;
  boost::optional<double> getSValue(
    geometry_msgs::msg::Point position,
    double threadhold_distance = 3.0,
    unsigned int initial_resolution = 30,
    unsigned int max_iteration = 30,
    double torelance = 0.001);
  double getSquaredDistanceIn2D(
    geometry_msgs::msg::Point point, double s) const;
  boost::optional<double> getCollisionPointIn2D(
    geometry_msgs::msg::Point point0,
    geometry_msgs::msg::Point point1,
    bool search_backward = false
  ) const;
  boost::optional<double> getCollisionPointIn2D(
    std::vector<geometry_msgs::msg::Point> polygon,
    bool search_backward = false
  ) const;
  const openscenario_msgs::msg::CatmullRomSpline toRosMsg() const;
  const geometry_msgs::msg::Point getRightBoundsPoint(
    double width, double s,
    double z_offset = 0) const;
  const geometry_msgs::msg::Point getLeftBoundsPoint(
    double width, double s,
    double z_offset = 0) const;
  const std::vector<geometry_msgs::msg::Point> getPolygon(
    double width, size_t num_points = 30, double z_offset = 0);

private:
  const std::vector<geometry_msgs::msg::Point> getRightBounds(
    double width,
    size_t num_points = 30,
    double z_offset = 0) const;
  const std::vector<geometry_msgs::msg::Point> getLeftBounds(
    double width,
    size_t num_points = 30,
    double z_offset = 0) const;
  double getSInSplineCurve(size_t curve_index, double s) const;
  std::pair<size_t, double> getCurveIndexAndS(double s) const;
  bool checkConnection() const;
  bool equals(geometry_msgs::msg::Point p0, geometry_msgs::msg::Point p1) const;
  std::vector<HermiteCurve> curves_;
  std::vector<double> length_list_;
  std::vector<double> maximum_2d_curvatures_;
  double total_length_;
  const std::vector<geometry_msgs::msg::Point> control_points;
};
}  // namespace math
}  // namespace simulation_api

#endif  // SIMULATION_API__MATH__CATMULL_ROM_SPLINE_HPP_
