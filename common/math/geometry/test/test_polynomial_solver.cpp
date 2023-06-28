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

#include <gtest/gtest.h>

#include <geometry/solver/polynomial_solver.hpp>
#include <geometry/spline/hermite_curve.hpp>
#include <scenario_simulator_exception/exception.hpp>

constexpr double solver_tolerance = math::geometry::PolynomialSolver::tolerance;

bool checkValueWithTolerance(double value, double expected, double tolerance = solver_tolerance)
{
  if (tolerance < 0) {
    throw std::logic_error("tolerance should be over 0");
  }
  if (std::fabs(value - expected) < tolerance) {
    return true;
  }
  return false;
}

/// @note Testcase for ax+b
TEST(PolynomialSolverTest, LinearFunction)
{
  math::geometry::PolynomialSolver solver;
  EXPECT_DOUBLE_EQ(solver.linear(1, 1, 2), 3);  // x + 1 = 3 (x=2)
  EXPECT_DOUBLE_EQ(solver.linear(1, 0, 2), 2);  // x + 0 = 2 (x=2)
  EXPECT_DOUBLE_EQ(solver.linear(0, 0, 2), 0);  //     0 = 0 (x=2)
}

/**
 * @note Testcase for ax+b = 0
 * Coverage verification of each coefficient for the cubic equation ax^2 + bx + c = 0 in the range [-20, 20] in increments of 0.1
 */
TEST(PolynomialSolverTest, SolveLinearEquation)
{
  constexpr double min_value = 0;
  constexpr double max_value = 1;
  math::geometry::PolynomialSolver solver;
  for (double a = -20; a < 20; a = a + 0.1) {
    for (double b = -20; b < 20; b = b + 0.1) {
      /**
       * @note If the ax+b=0 (a=0,b=0), any x value is a solution,
       * so throwing a common::SimulationError error is expected.
       */
      if (std::abs(a) <= solver_tolerance && std::abs(b) <= solver_tolerance) {
        EXPECT_THROW(
          solver.solveLinearEquation(a, b, min_value, max_value), common::SimulationError);
      }
      /// @note Other cases, solver must be able to obtain solutions.
      else {
        auto ret = solver.solveLinearEquation(a, b, min_value, max_value);
        for (const auto & solution : ret) {
          EXPECT_TRUE(checkValueWithTolerance(solver.linear(a, b, solution), 0.0));
          EXPECT_TRUE(min_value <= solution && solution <= max_value);
        }
      }
    }
  }
}

/// @note Testcase for ax^2+bx+c
TEST(PolynomialSolverTest, QuadraticFunction)
{
  math::geometry::PolynomialSolver solver;
  EXPECT_DOUBLE_EQ(solver.quadratic(1, 1, 1, 2), 7);   //  x^2 + x + 1 =  7 (x=2)
  EXPECT_DOUBLE_EQ(solver.quadratic(1, 1, 0, 2), 6);   //  x^2 + x + 0 =  6 (x=2)
  EXPECT_DOUBLE_EQ(solver.quadratic(2, 0, 3, 2), 11);  // 2x^2     + 3 = 11 (x=2)
  EXPECT_DOUBLE_EQ(solver.quadratic(0, 0, 0, 2), 0);   //            0 =  0 (x=2)
}

/**
 * @note Testcase for ax^2+bx+c = 0
 * Coverage verification of each coefficient for the cubic equation ax^2 + bx + c = 0 in the range [-20, 20] in increments of 1
 */
TEST(PolynomialSolverTest, SolveQuadraticEquation)
{
  constexpr double min_value = 0;
  constexpr double max_value = 1;
  math::geometry::PolynomialSolver solver;
  for (double a = -20; a < 20; a = a + 1) {
    for (double b = -20; b < 20; b = b + 1) {
      for (double c = -20; c < 20; c = c + 1) {
        /**
         * @note If the ax^2+bx+c=0 (a=0,b=0,c=0), any x value is a solution,
         * so throwing a common::SimulationError error is expected.
         */
        if (
          std::abs(a) <= solver_tolerance && std::abs(b) <= solver_tolerance &&
          std::abs(c) <= solver_tolerance) {
          EXPECT_THROW(
            solver.solveQuadraticEquation(a, b, c, min_value, max_value), common::SimulationError);
        }
        /// @note Other cases, solver must be able to obtain solutions.
        else {
          auto ret = solver.solveQuadraticEquation(a, b, c, 0, 1);
          for (const auto & solution : ret) {
            EXPECT_TRUE(checkValueWithTolerance(solver.quadratic(a, b, c, solution), 0.0));
            EXPECT_TRUE(min_value <= solution && solution <= max_value);
          }
        }
      }
    }
  }
}

/// @note Testcase for ax^3+bx^2+cx+d
TEST(PolynomialSolverTest, CubicFunction)
{
  math::geometry::PolynomialSolver solver;
  EXPECT_DOUBLE_EQ(solver.cubic(1, 1, 1, 1, 2), 15);  //  x^3 +  x^2 + x + 1 = 15 (x=2)
  EXPECT_DOUBLE_EQ(solver.cubic(4, 1, 1, 0, 2), 38);  // 4x^3 +  x^2 + x + 0 = 38 (x=2)
  EXPECT_DOUBLE_EQ(solver.cubic(3, 2, 0, 3, 2), 35);  // 3x^3 + 2x^2     + 3 = 35 (x=2)
  EXPECT_DOUBLE_EQ(solver.cubic(0, 0, 0, 0, 2), 0);   //                   0 =  0 (x=2)
}

/// @note Testcase for ax^3+bx^2+cx+d = 0
TEST(PolynomialSolverTest, SolveSpecificCubicEquation)
{
  constexpr double infinity = std::numeric_limits<double>::infinity();
  math::geometry::PolynomialSolver solver;
  /// @note solve x^3 - 2x^2 - 11x + 12 = 0 (solutions should be -3, 1, 4)
  auto solutions = solver.solveCubicEquation(1, -2, -11, 12, -infinity, infinity);
  std::sort(solutions.begin(), solutions.end());
  EXPECT_EQ(static_cast<int>(solutions.size()), 3);
  if (solutions.size() == 3) {
    EXPECT_TRUE(checkValueWithTolerance(solutions[0], -3));
    EXPECT_TRUE(checkValueWithTolerance(solutions[1], 1));
    EXPECT_TRUE(checkValueWithTolerance(solutions[2], 4));
  }
}

/// @note Testcase for ax^3+bx^2+cx+d = 0
TEST(PolynomialSolverTest, SolveSpecificCubicEquationWithMinMax)
{
  constexpr double min_value = 0;
  constexpr double max_value = 1;
  /**
   * @note solve x^3 - 2x^2 - 11x + 12 = 0 (solutions should be 1)
   * range of the solution should be [min_value, max_value].
   */
  math::geometry::PolynomialSolver solver;
  auto solutions = solver.solveCubicEquation(1, -2, -11, 12, min_value, max_value);
  EXPECT_EQ(static_cast<int>(solutions.size()), 1);
  if (solutions.size() == 1) {
    /**
     * @note I used EXPECT_TRUE(checkValueWithTolerance(solutions[1], 1)) in 1 = max_value, SolveSpecificCubicEquation testcase,
     * but in this test case, The solution x obtained satisfies std::abs(x-max_value) <= solver_tolerance, 
     * so the value of max_value (in this case, double 1.0 value) must be assigned.
     */
    EXPECT_DOUBLE_EQ(solutions[0], 1);
  }
}

/**
 * @note Testcase for ax^3+bx^2+cx+d = 0
 * Coverage verification of each coefficient for the cubic equation ax^3 + bx^2 + cx + d = 0 in the range [-10, 10] in increments of 1
 */
TEST(PolynomialSolverTest, SolveCubicEquation)
{
  constexpr double min_value = 0;
  constexpr double max_value = 1;
  math::geometry::PolynomialSolver solver;
  for (double a = -10; a < 10; a = a + 1) {
    for (double b = -10; b < 10; b = b + 1) {
      for (double c = -10; c < 10; c = c + 1) {
        for (double d = -10; d < 10; d = d + 1) {
          /**
           * @note If the ax^3+bx^2+cx+d=0 (a=0,b=0,c=0,d=0), any x value is a solution,
           * so throwing a common::SimulationError error is expected.
           */
          if (
            std::abs(a) <= solver_tolerance && std::abs(b) <= solver_tolerance &&
            std::abs(c) <= solver_tolerance && std::abs(d) <= solver_tolerance) {
            EXPECT_THROW(
              solver.solveCubicEquation(a, b, c, d, min_value, max_value), common::SimulationError);
          }
          /// @note Other cases, solver must be able to obtain solutions.
          else {
            auto ret = solver.solveCubicEquation(a, b, c, d, min_value, max_value);
            for (const auto & solution : ret) {
              EXPECT_TRUE(checkValueWithTolerance(solver.cubic(a, b, c, d, solution), 0.0));
              EXPECT_TRUE(min_value <= solution && solution <= max_value);
            }
          }
        }
      }
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
