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

// headers in Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

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
  ) const;

private:
  double getNewtonMethodStepSize(
    geometry_msgs::msg::Point point, double s,
    bool autoscale = false) const;
  /**
   * @brief calculate result of cubic function a*t^3 + b*t^2 + c*t + d
   *
   * @param a
   * @param b
   * @param c
   * @param d
   * @param t
   * @return double result of cubic function
   */
  inline double cubicFunction(double a, double b, double c, double d, double t) const
  {
    return a * t * t * t + b * t * t + c * t + d;
  }
  /**
   * @brief calculate result of quadratic function a*t^2 + b*t + c
   *
   * @param a
   * @param b
   * @param c
   * @param t
   * @return double result of quadratic function
   */
  inline double quadraticFunction(double a, double b, double c, double t) const
  {
    return a * t * t + b * t + c;
  }
  /**
   * @brief solve cubic equation x^3 + a*x^2 + b*x + c = 0, this code is public domain
   * @sa http://math.ivanovo.ac.ru/dalgebra/Khashin/poly/index.html
   * @param x
   * @param a
   * @param b
   * @param c
   * @return int
             if return value is 3, 3 real roots: x[0], x[1], x[2],
             if return value is 2, 2 real roots: x[0], x[1],
             if return value is 1, 1 real root : x[0], x[1] ± i*x[2],
   */
  int solveP3(std::vector<double> & x, double a, double b, double c);
  double _root3(double x)
  {
    double s = 1.;
    while (x < 1.) {
      x *= 8.;
      s *= 0.5;
    }
    while (x > 8.) {
      x *= 0.125;
      s *= 2.;
    }
    double r = 1.5;
    r -= 1. / 3. * ( r - x / ( r * r ) );
    r -= 1. / 3. * ( r - x / ( r * r ) );
    r -= 1. / 3. * ( r - x / ( r * r ) );
    r -= 1. / 3. * ( r - x / ( r * r ) );
    r -= 1. / 3. * ( r - x / ( r * r ) );
    r -= 1. / 3. * ( r - x / ( r * r ) );
    return r * s;
  }

  double root3(double x)
  {
    if (x > 0) {return _root3(x);} else if (x < 0) {return -_root3(-x);} else {
      return 0.;
    }
  }

};
}  // namespace math
}  // namespace simulation_api

#endif  // SIMULATION_API__MATH__HERMITE_CURVE_HPP_
