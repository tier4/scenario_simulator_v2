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

#include <complex>
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
  auto solveLinearEquation(
    const double a, const double b, const double min_value = 0, const double max_value = 1) const
    -> std::vector<double>;
  /**
   * @brief solve quadratic equation a*x^2 + b*x + c = 0
   *
   * @param a
   * @param b
   * @return std::vector<double> real solution of the quadratic functions (from min_value to max_value)
   */
  auto solveQuadraticEquation(
    const double a, const double b, const double c, const double min_value = 0,
    const double max_value = 1) const -> std::vector<double>;
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
    const double a, const double b, const double c, const double d, const double min_value = 0,
    const double max_value = 1) const -> std::vector<double>;
  /**
   * @brief calculate result of linear function a*t + b
   *
   * @param a
   * @param b
   * @param t
   * @return double result of linear function
   */
  auto linear(const double a, const double b, const double t) const -> double;
  /**
   * @brief calculate result of quadratic function a*t^2 + b*t + c
   *
   * @param a
   * @param b
   * @param c
   * @param t
   * @return double result of quadratic function
   */
  auto quadratic(const double a, const double b, const double c, const double t) const -> double;
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
  auto cubic(const double a, const double b, const double c, const double d, const double t) const
    -> double;
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

  /**
   * @brief check the value0 and value1 is equal or not with considering tolerance.
   * @param value0 the value you want to compare
   * @param value1 the value you want to compared
   * @return true value0 and value1 are equal
   * @return false value0 and value1 are not equal
   */
  auto isApproximatelyEqualTo(const double value0, const double value1) const -> bool;

private:
  /**
   * @brief solve cubic equation x^3 + a*x^2 + b*x + c = 0
   * @param a 
   * @param b 
   * @param c 
   * @return std::vector<std::complex<double>> Up to 3 complex solutions
   */
  auto solveMonicCubicEquationWithComplex(const double a, const double b, const double c) const
    -> std::vector<std::complex<double>>;
  /**
   * @brief filter values by range.
   * @param values the values you want to check.
   * @return std::vector<double> filtered values.
   */
  auto filterByRange(
    const std::vector<double> & values, const double min_value, const double max_value) const
    -> std::vector<double>;
};
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__SOLVER__POLYNOMIAL_SOLVER_HPP_
