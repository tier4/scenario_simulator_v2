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

#ifndef TRAFFIC_SIMULATOR__MATH__HERMITE_CURVE_HPP_
#define TRAFFIC_SIMULATOR__MATH__HERMITE_CURVE_HPP_

#include <gtest/gtest.h>
#include <quaternion_operation/quaternion_operation.h>

#include <boost/optional.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <traffic_simulator/math/polynomial_solver.hpp>
#include <vector>

namespace traffic_simulator
{
namespace math
{
class HermiteCurve
{
private:
  friend class HermiteCurveTest;
  double ax_, bx_, cx_, dx_;
  double ay_, by_, cy_, dy_;
  double az_, bz_, cz_, dz_;
  traffic_simulator::math::PolynomialSolver solver_;

public:
  HermiteCurve(
    geometry_msgs::msg::Pose start_pose, geometry_msgs::msg::Pose goal_pose,
    geometry_msgs::msg::Vector3 start_vec, geometry_msgs::msg::Vector3 goal_vec);
  HermiteCurve(
    double ax, double bx, double cx, double dx, double ay, double by, double cy, double dy,
    double az, double bz, double cz, double dz);
  std::vector<geometry_msgs::msg::Point> getTrajectory(size_t num_points = 30) const;
  const std::vector<geometry_msgs::msg::Point> getTrajectory(
    double start_s, double end_s, double resolution, bool autoscale = false) const;
  const geometry_msgs::msg::Pose getPose(double s, bool autoscale = false) const;
  const geometry_msgs::msg::Point getPoint(double s, bool autoscale = false) const;
  const geometry_msgs::msg::Vector3 getTangentVector(double s, bool autoscale = false) const;
  const geometry_msgs::msg::Vector3 getNormalVector(double s, bool autoscale = false) const;
  double get2DCurvature(double s, bool autoscale = false) const;
  double getMaximum2DCurvature() const;
  double getLength(size_t num_points) const;
  double getLength() const { return length_; }
  boost::optional<double> getSValue(
    const geometry_msgs::msg::Pose & pose, double threshold_distance = 3.0,
    bool autoscale = false) const;
  double getSquaredDistanceIn2D(
    geometry_msgs::msg::Point point, double s, bool autoscale = false) const;
  boost::optional<double> getCollisionPointIn2D(
    const geometry_msgs::msg::Point & point0, const geometry_msgs::msg::Point & point1,
    bool search_backward = false) const;
  boost::optional<double> getCollisionPointIn2D(
    const std::vector<geometry_msgs::msg::Point> & polygon, bool search_backward = false,
    bool close_start_end = true) const;

private:
  std::pair<double, double> get2DMinMaxCurvatureValue() const;
  double length_;
};
}  // namespace math
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__MATH__HERMITE_CURVE_HPP_
