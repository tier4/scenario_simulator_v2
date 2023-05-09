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
double PolynomialSolver::linearFunction(double a, double b, double t) const { return a * t + b; }

double PolynomialSolver::cubicFunction(double a, double b, double c, double d, double t) const
{
  return a * t * t * t + b * t * t + c * t + d;
}

double PolynomialSolver::quadraticFunction(double a, double b, double c, double t) const
{
  return a * t * t + b * t + c;
}

std::vector<double> PolynomialSolver::solveLinearEquation(
  double a, double b, double min_value, double max_value) const
{
  if (std::abs(a) <= tolerance) {
    if (std::abs(b) <= tolerance) {
      THROW_SIMULATION_ERROR(
        "Not computable because a=0 in the linear equation ", a, " x + ", b,
        "=0, "
        "so any value of x = ",
        min_value, "~", max_value, " will be the solution.",
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

std::vector<double> PolynomialSolver::solveQuadraticEquation(
  double a, double b, double c, double min_value, double max_value) const
{
  std::vector<double> candidates, ret;
  if (std::abs(a) <= tolerance) {
    return solveLinearEquation(b, c);
  }
  double root = b * b - 4 * a * c;
  if (std::abs(root) <= tolerance) {
    candidates = {-b / (2 * a)};
  } else if (root < 0) {
    candidates = {};
  } else {
    candidates = {(-b - std::sqrt(root)) / (2 * a), (-b + std::sqrt(root)) / (2 * a)};
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

std::vector<double> PolynomialSolver::solveCubicEquation(
  double a, double b, double c, double d, double min_value, double max_value) const
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

int PolynomialSolver::solveP3(std::vector<double> & x, double a, double b, double c) const
{
  x = std::vector<double>(3);
  double a2 = a * a;
  /**
   * @note Tschirnhaus transformation, transform into x^3 + q*x + r = 0
   * @sa https://science-log.com/%E6%95%B0%E5%AD%A6/3%E6%AC%A1%E6%96%B9%E7%A8%8B%E5%BC%8F%E3%81%AE%E8%A7%A3%E3%81%AE%E5%85%AC%E5%BC%8F/
   */
  double q = (a2 - 3 * b) / 9;
  double r = (a * (2 * a2 - 9 * b) + 27 * c) / 54;
  double r2 = r * r;
  double q3 = q * q * q;
  double A, B;
  if (r2 <= (q3 + tolerance)) {
    /**
     * @note If 3 real roots are found.
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
     * @note If the imaginary part of the complex almost zero, this equation has a multiple root.
     */
    if (std::abs(x[2]) <= tolerance) {
      x[2] = x[1];
      return 2;
    }
    return 1;
  }
  /**
   * @note No roots are found.
   */
  return 0;
}
}  // namespace geometry
}  // namespace math
