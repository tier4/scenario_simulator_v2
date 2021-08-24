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

#include <gtest/gtest.h>

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/math/hermite_curve.hpp>
#include <traffic_simulator/math/polynomial_solver.hpp>

TEST(PolynomialSolverTest, SolveLinearEquation)
{
  traffic_simulator::math::PolynomialSolver solver;
  auto ret = solver.solveLinearEquation(-20, 3, 0, 1);
  EXPECT_EQ(ret.size(), static_cast<size_t>(1));
  EXPECT_DOUBLE_EQ(ret[0], 0.15);
}

TEST(PolynomialSolverTest, SolveQuadraticEquation)
{
  traffic_simulator::math::PolynomialSolver solver;
  auto ret = solver.solveQuadraticEquation(2, 3, -5, 0, 2);
  EXPECT_EQ(ret.size(), static_cast<size_t>(1));
  EXPECT_DOUBLE_EQ(ret[0], 1);
}

TEST(PolynomialSolverTest, SolveCubicEquation)
{
  traffic_simulator::math::PolynomialSolver solver;
  auto ret = solver.solveCubicEquation(1, -2, -11, 12, 0, 2);
  EXPECT_EQ(ret.size(), static_cast<size_t>(1));
  EXPECT_DOUBLE_EQ(ret[0], 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
