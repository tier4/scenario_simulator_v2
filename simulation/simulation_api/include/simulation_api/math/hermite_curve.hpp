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

#ifndef SIMULATION_API__MATH__HERMITE_CURVE_HPP_
#define SIMULATION_API__MATH__HERMITE_CURVE_HPP_

#include <quaternion_operation/quaternion_operation.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <boost/optional.hpp>

#include <vector>

namespace simulation_api
{
namespace math
{
class HermiteCurve
{
private:
  double ax_, bx_, cx_, dx_;
  double ay_, by_, cy_, dy_;
  double az_, bz_, cz_, dz_;

public:
  HermiteCurve(
    geometry_msgs::msg::Pose start_pose, geometry_msgs::msg::Pose goal_pose,
    geometry_msgs::msg::Vector3 start_vec, geometry_msgs::msg::Vector3 goal_vec);
  HermiteCurve(
    double ax, double bx, double cx, double dx,
    double ay, double by, double cy, double dy,
    double az, double bz, double cz, double dz);
  std::vector<geometry_msgs::msg::Point> getTrajectory() const;
  const geometry_msgs::msg::Pose getPose(
    double s,
    bool autoscale = false) const;
  const geometry_msgs::msg::Point getPoint(
    double s,
    bool autoscale = false) const;
  const geometry_msgs::msg::Vector3 getTangentVector(
    double s,
    bool autoscale = false) const;
  double get2DCurvature(
    double s,
    bool autoscale = false) const;
  double getMaximu2DCurvature() const;
  double getLength() const;
  boost::optional<double> getSValue(
    geometry_msgs::msg::Point position,
    double threadhold_distance = 1.0,
    unsigned int initial_resolution = 30,
    unsigned int max_iteration = 30,
    double torelance = 0.001,
    bool autoscale = false) const;
  double getSquaredDistanceIn2D(
    geometry_msgs::msg::Point point, double s,
    bool autoscale = false) const;
  boost::optional<double> getCollisionPointIn2D(
    geometry_msgs::msg::Point point0,
    geometry_msgs::msg::Point point1
  );

private:
  double getNewtonMethodStepSize(
    geometry_msgs::msg::Point point, double s,
    bool autoscale = false) const;
  /**
   * @brief slove quadratic equation 0 = a*x^2 + b*x + c
   *
   * @param a coefficients of quadratic equation
   * @param b coefficients of quadratic equation
   * @param c coefficients of quadratic equation
   * @param min_solution minimum values of solutions
   * @param max_solution maximum values of solutions
   * @return std::vector<double> array of real solutions
   */
  std::vector<double> solveQuadraticEquation(
    double a, double b, double c, double min_solution = 0,
    double max_solution = 1) const;
  std::vector<double> solveTwoCubicEquations(
    double a0, double b0, double c0, double d0,
    double a1, double b1, double c1, double d1,
    double min_solution = 0,
    double max_solution = 1) const;
  double quadraticFunction(double a, double b, double c, double s) const;
  double cubicFunction(double a, double b, double c, double d, double s) const;
};
}  // namespace math
}  // namespace simulation_api

#endif  // SIMULATION_API__MATH__HERMITE_CURVE_HPP_
