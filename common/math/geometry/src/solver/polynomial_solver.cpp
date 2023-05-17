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

#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <geometry/solver/polynomial_solver.hpp>
#include <iostream>
#include <limits>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <vector>

namespace math
{
namespace geometry
{
auto PolynomialSolver::linear(double a, double b, double t) const -> double { return a * t + b; }

auto PolynomialSolver::quadratic(double a, double b, double c, double t) const -> double
{
  return a * t * t + b * t + c;
}

auto PolynomialSolver::cubic(double a, double b, double c, double d, double t) const -> double
{
  return a * t * t * t + b * t * t + c * t + d;
}

auto PolynomialSolver::solveLinearEquation(
  double a, double b, double min_value, double max_value) const -> std::vector<double>
{
  const auto solve_without_limit = [this](double a, double b) -> std::vector<double> {
    /// @note In this case, ax*b = 0 (a=0) can cause division by zero. So give special treatment to this case.
    if (isEqual(a, 0)) {
      if (isEqual(b, 0)) {
        THROW_SIMULATION_ERROR(
          "Not computable x because of the linear equation ", a, " x + ", b, "=0, and a = ", a,
          ", b = ", b, " is very close to zero ,so any value of x will be the solution.",
          "There are no expected cases where this exception is thrown.",
          "Please contact the scenario_simulator_v2 developers, ",
          "especially Masaya Kataoka (@hakuturu583).");
      }
      /// @note In this case, ax*b = 0 (a=0,b!=0) so any x cannot satisfy this equation.
      return {};
    }
    /// @note In this case, ax*b = 0 (a!=0, b!=0) so x = -b/a is a only solution.
    return {-b / a};
  };

  /// @note No fallback because of the order cannot be lowered any further.
  return filterByRange(solve_without_limit(a, b), min_value, max_value);
}

auto PolynomialSolver::solveQuadraticEquation(
  double a, double b, double c, double min_value, double max_value) const -> std::vector<double>
{
  const auto solve_without_limit = [this](double a, double b, double c) -> std::vector<double> {
    if (double discriminant = b * b - 4 * a * c; isEqual(discriminant, 0)) {
      return {-b / (2 * a)};
    } else if (discriminant < 0) {
      return {};
    } else {
      return {(-b - std::sqrt(discriminant)) / (2 * a), (-b + std::sqrt(discriminant)) / (2 * a)};
    }
  };

  /// @note Fallback to linear equation solver if a = 0
  return isEqual(a, 0) ? solveLinearEquation(b, c, min_value, max_value)
                       : filterByRange(solve_without_limit(a, b, c), min_value, max_value);
}

auto PolynomialSolver::solveCubicEquation(
  double a, double b, double c, double d, double min_value, double max_value) const
  -> std::vector<double>
{
  const auto solve_without_limit = [this](double a, double b, double c, double d) {
    /// @note Function that takes a std::vector of complex numbers and selects only real numbers from it and returns them
    const auto get_real_values =
      [](const std::vector<std::complex<double>> & complex_values) -> std::vector<double> {
      /**
       * @note Function that takes a complex number as input and returns the real part if it is a real number (imaginary part is 0) 
       * or std::nullopt if it is an imaginary or complex number.
       */
      const auto is_real_value = [](const std::complex<double> & complex_value) {
        constexpr double epsilon = std::numeric_limits<double>::epsilon();
        return (std::abs(complex_value.imag()) <= epsilon)
                 ? std::optional<double>(complex_value.real())
                 : std::nullopt;
      };
      /// @note Iterate all complex values and check the value is real value or not.
      std::vector<double> real_values = {};
      std::for_each(
        complex_values.begin(), complex_values.end(),
        [&real_values, is_real_value](const auto complex_value) mutable {
          if (const auto real_value = is_real_value(complex_value)) {
            real_values.push_back(real_value.value());
          }
        });
      return real_values;
    };
    return get_real_values(solveMonicCubicEquationWithComplex(b / a, c / a, d / a));
  };

  /// @note Fallback to quadratic equation solver if a = 0
  return isEqual(a, 0) ? solveQuadraticEquation(b, c, d, min_value, max_value)
                       : filterByRange(solve_without_limit(a, b, c, d), min_value, max_value);
}

auto PolynomialSolver::filterByRange(
  const std::vector<double> & values, const double min_value, const double max_value) const
  -> std::vector<double>
{
  /**
   * @note Function to check if value exists between [min_value,max_value] considering the tolerance,
   * returning std::nullopt if not present. If not, return std::nullopt, otherwise return value.
   */
  const auto is_in_range =
    [](double value, double min_value, double max_value) -> std::optional<double> {
    if (min_value <= value && value <= max_value) {
      return value;
    } else if (std::abs(value - max_value) <= tolerance) {
      return max_value;
    } else if (std::abs(value - min_value) <= tolerance) {
      return min_value;
    }
    return std::nullopt;
  };
  /// @note Iterate values and check the value is in range or not.
  std::vector<double> filtered_values = {};
  std::for_each(
    values.begin(), values.end(),
    [&filtered_values, is_in_range, min_value, max_value](const double value) mutable {
      if (const auto filtered_value = is_in_range(value, min_value, max_value)) {
        filtered_values.push_back(filtered_value.value());
      }
    });
  return filtered_values;
}

/// @note this code is public domain (http://math.ivanovo.ac.ru/dalgebra/Khashin/poly/index.html)
auto PolynomialSolver::solveMonicCubicEquationWithComplex(
  const double a, const double b, const double c) const -> std::vector<std::complex<double>>
{
  const double a2 = a * a;
  /**
   * @note Tschirnhaus transformation, transform into x^3 + 3q*x + 2r = 0
   * @sa https://oshima-gakushujuku.com/blog/math/formula-qubic-equation/
   */
  const double q = (a2 - 3 * b) / 9;
  const double r = (a * (2 * a2 - 9 * b) + 27 * c) / 54;
  const double q3 = q * q * q;
  if (r * r <= (q3 + tolerance)) {
    /**
     * @note If 3 real solutions are found.
     * The URL specified in @sa is a reference material for developers who wish to follow the formulas,
     * and the code that exists in the material is not included in this library.
     * @sa https://onihusube.hatenablog.com/entry/2018/10/08/140426
     */
    const double t = std::acos(std::clamp(r / std::sqrt(q3), -1.0, 1.0));
    return {
      // clang-format off
      std::complex<double>(-2 * std::sqrt(q) * std::cos(t / 3) - a / 3, 0),
      std::complex<double>(-2 * std::sqrt(q) * std::cos((t + boost::math::constants::two_pi<double>()) / 3) - a / 3, 0),
      std::complex<double>(-2 * std::sqrt(q) * std::cos((t - boost::math::constants::two_pi<double>()) / 3) - a / 3, 0)
      // clang-format on
    };
  } else {
    /**
     * @note If imaginary solutions exist.
     */
    const double A = [r, q3]() {
      const auto calculate_real_solution = [r, q3]() {
        return -std::cbrt(std::abs(r) + std::sqrt(r * r - q3));
      };
      return r < 0 ? -1 * calculate_real_solution() : calculate_real_solution();
    }();
    const double B = isEqual(A, 0) ? 0 : q / A;
    /**
     * @note If the imaginary part of the complex almost zero, this equation has a multiple solution.
     */
    const double imaginary_part = 0.5 * std::sqrt(3.0) * (A - B);
    if (isEqual(imaginary_part, 0)) {
      return {
        // clang-format off
        std::complex<double>((A + B) - a / 3, 0),
        std::complex<double>(-0.5 * (A + B) - a / 3)
        // clang-format on
      };
    }
    return {
      // clang-format off
      std::complex<double>((A + B) - a / 3, 0),
      std::complex<double>(-0.5 * (A + B) - a / 3, -imaginary_part),
      std::complex<double>(-0.5 * (A + B) - a / 3,  imaginary_part)
      // clang-format on
    };
  }
  /**
   * @note No solutions are found.
   */
  return {};
}

auto PolynomialSolver::isEqual(double value0, double value1) const -> bool
{
  return std::abs(value0 - value1) <= tolerance;
}
}  // namespace geometry
}  // namespace math
