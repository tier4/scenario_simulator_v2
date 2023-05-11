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

#include <cmath>
#include <geometry/solver/polynomial_solver.hpp>
#include <iostream>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <vector>

namespace math
{
namespace geometry
{
auto PolynomialSolver::linearFunction(double a, double b, double t) const -> double
{
  return a * t + b;
}

auto PolynomialSolver::quadraticFunction(double a, double b, double c, double t) const -> double
{
  return a * t * t + b * t + c;
}

auto PolynomialSolver::cubicFunction(double a, double b, double c, double d, double t) const
  -> double
{
  return a * t * t * t + b * t * t + c * t + d;
}

auto PolynomialSolver::solveLinearEquation(
  double a, double b, double min_value, double max_value) const -> std::vector<double>
{
  /**
   * @note In this case, ax*b = 0 (a=0) can cause division by zero.
   * So give special treatment to this case.
   */
  if (std::abs(a) <= tolerance) {
    if (std::abs(b) <= tolerance) {
      THROW_SIMULATION_ERROR(
        "Not computable x because a=0, b=0 in the linear equation ", a, " x + ", b,
        "=0, so any value of x = ", min_value, "~", max_value, " will be the solution.",
        "There are no expected cases where this exception is thrown.",
        "Please contact the scenario_simulator_v2 developers, ",
        "especially Masaya Kataoka (@hakuturu583).");
    }
    /**
     * @note In this case, ax*b = 0 (a=0, b≠0) so any x cannot satisfy this equation.
     */
    return {};
  }
  double ret = -b / a;
  if (min_value <= ret && ret <= max_value) {
    return {ret};
  } else if (std::abs(ret - max_value) <= tolerance) {
    return {max_value};
  } else if (std::abs(ret - min_value) <= tolerance) {
    return {min_value};
  }
  return {};
}

auto PolynomialSolver::solveQuadraticEquation(
  double a, double b, double c, double min_value, double max_value) const -> std::vector<double>
{
  std::vector<double> candidates, ret;
  if (std::abs(a) <= tolerance) {
    return solveLinearEquation(b, c);
  }
  double discriminant = b * b - 4 * a * c;
  if (std::abs(discriminant) <= tolerance) {
    candidates = {-b / (2 * a)};
  } else if (discriminant < 0) {
    candidates = {};
  } else {
    candidates = {
      (-b - std::sqrt(discriminant)) / (2 * a), (-b + std::sqrt(discriminant)) / (2 * a)};
  }
  for (const auto candidate : candidates) {
    if (min_value <= candidate && candidate <= max_value) {
      ret.push_back(candidate);
    } else if (std::abs(candidate - max_value) <= tolerance) {
      ret.push_back(max_value);
    } else if (std::abs(candidate - min_value) <= tolerance) {
      ret.push_back(min_value);
    }
  }
  return ret;
}

auto PolynomialSolver::solveCubicEquation(
  double a, double b, double c, double d, double min_value, double max_value) const
  -> std::vector<double>
{
  if (std::abs(a) <= tolerance) {
    return solveQuadraticEquation(b, c, d);
  }
  std::vector<double> solutions, candidates, ret;
  auto result = solveP3(solutions, b / a, c / a, d / a);
  if (result == 3) {
    candidates = solutions;
  } else if (result == 2) {
    candidates = {solutions[0], solutions[1]};
  } else if (result == 1) {
    candidates = {solutions[0]};
  }
  for (const auto candidate : candidates) {
    if (min_value <= candidate && candidate <= max_value) {
      ret.push_back(candidate);
    } else if (std::abs(candidate - max_value) <= tolerance) {
      ret.push_back(max_value);
    } else if (std::abs(candidate - min_value) <= tolerance) {
      ret.push_back(min_value);
    }
  }
  return ret;
}

auto PolynomialSolver::solveP3(std::vector<double> & x, double a, double b, double c) const -> int
{
  x = std::vector<double>(3);
  double a2 = a * a;
  /**
   * @note Tschirnhaus transformation, transform into x^3 + 3q*x + 2r = 0
   * @sa https://oshima-gakushujuku.com/blog/math/formula-qubic-equation/
   */
  double q = (a2 - 3 * b) / 9;
  double r = (a * (2 * a2 - 9 * b) + 27 * c) / 54;
  double r2 = r * r;
  double q3 = q * q * q;
  double A, B;
  if (r2 <= (q3 + tolerance)) {
    /**
     * @note If 3 real solutions are found.
     * @sa https://onihusube.hatenablog.com/entry/2018/10/08/140426
     */
    double t = r / std::sqrt(q3);
    if (t < -1) {
      t = -1;
    }
    if (t > 1) {
      t = 1;
    }
    t = std::acos(t);
    a /= 3;
    q = -2 * std::sqrt(q);
    x[0] = q * std::cos(t / 3) - a;
    x[1] = q * std::cos((t + M_PI * 2) / 3) - a;
    x[2] = q * std::cos((t - M_PI * 2) / 3) - a;
    return 3;
  } else {
    /**
     * @note If imaginary solutions exist.
     */
    A = -std::cbrt(std::abs(r) + std::sqrt(r2 - q3));
    if (r < 0) {
      A = -A;
    }
    if (A == 0) {
      B = 0;
    } else {
      B = q / A;
    }
    a /= 3;
    x[0] = (A + B) - a;
    x[1] = -0.5 * (A + B) - a;
    x[2] = 0.5 * std::sqrt(3.) * (A - B);
    /**
     * @note If the imaginary part of the complex almost zero, this equation has a multiple solution.
     */
    if (std::abs(x[2]) <= tolerance) {
      x[2] = x[1];
      return 2;
    }
    return 1;
  }
  /**
   * @note No solutions are found.
   */
  return 0;
}
}  // namespace geometry
}  // namespace math
