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

#ifndef GEOMETRY__SOLVER__POLYNOMIAL_SOLVER_HPP_
#define GEOMETRY__SOLVER__POLYNOMIAL_SOLVER_HPP_

#include <vector>

namespace math
{
namespace geometry
{
class PolynomialSolver
{
public:
  /**
 * @brief solve linear equation a*x + b = 0
 *
 * @param a
 * @param b
 * @return std::vector<double> real solution of the quadratic functions (from min_value to max_value)
 */
  auto solveLinearEquation(double a, double b, double min_value = 0, double max_value = 1) const
    -> std::vector<double>;
  /**
  * @brief solve quadratic equation a*x^2 + b*x + c = 0
  *
  * @param a
  * @param b
  * @return std::vector<double> real solution of the quadratic functions (from min_value to max_value)
  */
  auto solveQuadraticEquation(
    double a, double b, double c, double min_value = 0, double max_value = 1) const
    -> std::vector<double>;
  /**
 * @brief solve cubic function a*t^3 + b*t^2 + c*t + d = 0
 *
 * @param a
 * @param b
 * @param c
 * @param d
 * @return std::vector<double> real solution of the cubic functions (from min_value to max_value)
 */
  auto solveCubicEquation(
    double a, double b, double c, double d, double min_value = 0, double max_value = 1) const
    -> std::vector<double>;
  /**
 * @brief calculate result of linear function a*t + b
 *
 * @param a
 * @param b
 * @param t
 * @return double result of linear function
 */
  auto linearFunction(double a, double b, double t) const -> double;
  /**
   * @brief calculate result of quadratic function a*t^2 + b*t + c
   *
   * @param a
   * @param b
   * @param c
   * @param t
   * @return double result of quadratic function
   */
  auto quadraticFunction(double a, double b, double c, double t) const -> double;
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
  auto cubicFunction(double a, double b, double c, double d, double t) const -> double;
  /**
   * @brief Hard coded parameter, tolerance of calculation results of the PolynomialSolver.
   * This value was determined by Masaya Kataoka (@hakuturu583).
   * The reason this value is not std::numeric_limits<double>::epsilon is that when using 
   * this set of functions to find the intersection of a Catmull-Rom spline curve and a line segment, 
   * it was confirmed that when the solution is very close to the endpoints of the Hermite curves 
   * that make up the Catmull-Rom spline, the solution could not calculated.
   * When considering a 1 km long Catmull-Rom spline (assuming a very long lanelet) with a tolerance of 1e-7, 
   * the error in calculating the intersection position in a tangentially distorted coordinate system was ±0.0001 m.
   * This value is sufficiently small that it was tentatively determined to be 1e-7.
   */
  constexpr static double tolerance = 1e-7;

private:
  /**
   * @brief solve cubic equation x^3 + a*x^2 + b*x + c = 0
   * @param x
   * @param a
   * @param b
   * @param c
   * @return int
             if return value is 3, 3 real solutions: x[0], x[1], x[2],
             if return value is 2, 2 real solutions: x[0], x[1],
             if return value is 1, 1 real solution : x[0], x[1] ± i*x[2],
   */
  auto solveP3(std::vector<double> & x, const double a, const double b, const double c) const
    -> int;
};
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__SOLVER__POLYNOMIAL_SOLVER_HPP_
