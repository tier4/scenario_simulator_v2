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

#include <geometry/spline/hermite_curve.hpp>

#include "../expect_eq_macros.hpp"
#include "../test_utils.hpp"

constexpr double EPS = 1e-3;

/// @brief Helper function generating straight line: p(0,0) v(1,0)-> p(1,0) v(1,0)
math::geometry::HermiteCurve makeLine1()
{
  geometry_msgs::msg::Pose start_pose = makePose(0.0, 0.0);
  geometry_msgs::msg::Pose goal_pose = makePose(1.0, 0.0);
  geometry_msgs::msg::Vector3 start_vec = makeVector(1.0, 0.0);
  geometry_msgs::msg::Vector3 goal_vec = makeVector(1.0, 0.0);
  return math::geometry::HermiteCurve(start_pose, goal_pose, start_vec, goal_vec);
}

/// @brief Helper function generating straight line: p(0,0) v(1,1)-> p(2,2) v(1,1)
math::geometry::HermiteCurve makeLine2()
{
  geometry_msgs::msg::Pose start_pose = makePose(0.0, 0.0);
  geometry_msgs::msg::Pose goal_pose = makePose(2.0, 2.0);
  geometry_msgs::msg::Vector3 start_vec = makeVector(1.0, 1.0);
  geometry_msgs::msg::Vector3 goal_vec = makeVector(1.0, 1.0);
  return math::geometry::HermiteCurve(start_pose, goal_pose, start_vec, goal_vec);
}

/// @brief Helper function generating curve: p(0,0) v(1,0)-> p(1,1) v(0,1)
math::geometry::HermiteCurve makeCurve1(bool concave_upward = true)
{
  geometry_msgs::msg::Pose start_pose = makePose(0.0, 0.0);
  geometry_msgs::msg::Pose goal_pose = makePose(1.0, 1.0);
  geometry_msgs::msg::Vector3 start_vec =
    concave_upward ? makeVector(1.0, 0.0) : makeVector(0.0, 1.0);
  geometry_msgs::msg::Vector3 goal_vec =
    concave_upward ? makeVector(0.0, 1.0) : makeVector(1.0, 0.0);
  return math::geometry::HermiteCurve(start_pose, goal_pose, start_vec, goal_vec);
}

/// @brief Helper function generating curve: p(0,0) v(1,0)-> p(1,-1) v(0,-1)
math::geometry::HermiteCurve makeCurve2()
{
  geometry_msgs::msg::Pose start_pose = makePose(0.0, 0.0);
  geometry_msgs::msg::Pose goal_pose = makePose(1.0, -1.0);
  geometry_msgs::msg::Vector3 start_vec = makeVector(1.0, 0.0);
  geometry_msgs::msg::Vector3 goal_vec = makeVector(0.0, -1.0);
  return math::geometry::HermiteCurve(start_pose, goal_pose, start_vec, goal_vec);
}

/// @brief Helper function generating curve: p(1,1) v(0,-1)-> p(0,0) v(-1,0)
math::geometry::HermiteCurve makeCurve3()
{
  geometry_msgs::msg::Pose start_pose = makePose(1.0, 1.0);
  geometry_msgs::msg::Pose goal_pose = makePose(0.0, 0.0);
  geometry_msgs::msg::Vector3 start_vec = makeVector(0.0, -1.0);
  geometry_msgs::msg::Vector3 goal_vec = makeVector(-1.0, 0.0);
  return math::geometry::HermiteCurve(start_pose, goal_pose, start_vec, goal_vec);
}

/// @brief Helper function generating curve: p(1,-1) v(0,1)-> p(0,0) v(-1,0)
math::geometry::HermiteCurve makeCurve4()
{
  geometry_msgs::msg::Pose start_pose = makePose(1.0, -1.0);
  geometry_msgs::msg::Pose goal_pose = makePose(0.0, 0.0);
  geometry_msgs::msg::Vector3 start_vec = makeVector(0.0, 1.0);
  geometry_msgs::msg::Vector3 goal_vec = makeVector(-1.0, 0.0);
  return math::geometry::HermiteCurve(start_pose, goal_pose, start_vec, goal_vec);
}

/**
 * @brief Helper function generating a reference trajectory for testing
 * @param start_x starting X position
 * @param start_y starting Y position
 * @param increment_x increment over X axis on every step
 * @param increment_y increment over Y axis on every step
 * @param vec vector container to generate a reference trajectory in (has to be the size of desired trajectory)
 * @param start_idx index from which to start
 */
void generateReferenceTrajectory(
  double start_x, double start_y, double increment_x, double increment_y,
  std::vector<geometry_msgs::msg::Point> & vec, unsigned int start_idx = 0u)
{
  for (size_t i = 0; i < vec.size(); ++i) {
    vec[i].x = start_x + increment_x * (i + start_idx);
    vec[i].y = start_y + increment_y * (i + start_idx);
  }
}

TEST(HermiteCurveTest, initializationLine)
{
  geometry_msgs::msg::Pose start_pose = makePose(0.0, 0.0);
  geometry_msgs::msg::Pose goal_pose = makePose(1.0, 0.0);
  geometry_msgs::msg::Vector3 start_vec = makeVector(1.0, 0.0);
  geometry_msgs::msg::Vector3 goal_vec = makeVector(1.0, 0.0);
  EXPECT_NO_THROW(math::geometry::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec));
}

TEST(HermiteCurveTest, initializationCurve)
{
  geometry_msgs::msg::Pose start_pose = makePose(0.0, 0.0);
  geometry_msgs::msg::Pose goal_pose = makePose(1.0, 1.0);
  geometry_msgs::msg::Vector3 start_vec = makeVector(1.0, 0.0);
  geometry_msgs::msg::Vector3 goal_vec = makeVector(0.0, 1.0);
  EXPECT_NO_THROW(math::geometry::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec));
}

TEST(HermiteCurveTest, initializationParams)
{
  EXPECT_NO_THROW(math::geometry::HermiteCurve curve(
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0));
}

TEST(HermiteCurveTest, getTrajectoryZero)
{
  const auto curve = makeLine1();
  EXPECT_NO_THROW(auto trajectory = curve.getTrajectory(0));
  auto trajectory = curve.getTrajectory(0);
  EXPECT_EQ(trajectory.size(), size_t(0));
}

TEST(HermiteCurveTest, getTrajectory)
{
  const auto curve = makeLine1();
  EXPECT_NO_THROW(auto trajectory = curve.getTrajectory(3));
  auto trajectory = curve.getTrajectory(3);
  EXPECT_EQ(trajectory.size(), size_t(3));
  EXPECT_POINT_EQ(trajectory[0], makePoint(0.0, 0.0));
  EXPECT_POINT_EQ(trajectory[1], makePoint(0.5, 0.0));
  EXPECT_POINT_EQ(trajectory[2], makePoint(1.0, 0.0));
}

TEST(HermiteCurveTest, getTrajectoryReversed)
{
  geometry_msgs::msg::Pose start_pose = makePose(1.0, 0.0);
  geometry_msgs::msg::Pose goal_pose = makePose(0.0, 0.0);
  geometry_msgs::msg::Vector3 start_vec = makeVector(1.0, 0.0);
  geometry_msgs::msg::Vector3 goal_vec = makeVector(1.0, 0.0);
  math::geometry::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);
  EXPECT_NO_THROW(auto trajectory = curve.getTrajectory(3));
  auto trajectory = curve.getTrajectory(3);
  EXPECT_EQ(trajectory.size(), size_t(3));
  EXPECT_POINT_EQ(trajectory[0], makePoint(1.0, 0.0));
  EXPECT_POINT_EQ(trajectory[1], makePoint(0.5, 0.0));
  EXPECT_POINT_EQ(trajectory[2], makePoint(0.0, 0.0));
}

TEST(HermiteCurveTest, getTrajectoryPast1)
{
  const auto curve = makeLine2();
  EXPECT_NO_THROW(auto trajectory = curve.getTrajectory(0.0, 1.0, 0.1, false));
  auto trajectory = curve.getTrajectory(0.0, 1.0, 0.1, false);
  std::vector<geometry_msgs::msg::Point> ans(11);
  generateReferenceTrajectory(0.0, 0.0, 0.2, 0.2, ans, 1u);

  EXPECT_EQ(trajectory.size(), ans.size());
  // The uncertainty of the calculations is high because the parameter space is equidistant
  // interpolated, which does not translate into equidistant spline interpolation
  constexpr double eps = 0.15;
  EXPECT_POINT_NEAR(trajectory[0], ans[0], eps);
  EXPECT_POINT_NEAR(trajectory[1], ans[1], eps);
  EXPECT_POINT_NEAR(trajectory[2], ans[2], eps);
  EXPECT_POINT_NEAR(trajectory[3], ans[3], eps);
  EXPECT_POINT_NEAR(trajectory[4], ans[4], eps);
  EXPECT_POINT_NEAR(trajectory[5], ans[5], eps);
  EXPECT_POINT_NEAR(trajectory[6], ans[6], eps);
  EXPECT_POINT_NEAR(trajectory[7], ans[7], eps);
  EXPECT_POINT_NEAR(trajectory[8], ans[8], eps);
  EXPECT_POINT_NEAR(trajectory[9], ans[9], eps);
  EXPECT_POINT_NEAR(trajectory[10], ans[10], eps);
}

TEST(HermiteCurveTest, getTrajectoryPast2)
{
  const auto curve = makeLine2();
  EXPECT_NO_THROW(auto trajectory = curve.getTrajectory(0.0, std::sqrt(2.0) * 2.0, 0.1, true));
  auto trajectory = curve.getTrajectory(0, std::sqrt(2.0) * 2.0, 0.1 * std::sqrt(2.0) * 2.0, true);
  std::vector<geometry_msgs::msg::Point> ans(11);
  generateReferenceTrajectory(0.0, 0.0, 0.2, 0.2, ans, 1u);

  EXPECT_EQ(trajectory.size(), ans.size());
  // The uncertainty of the calculations is high because the parameter space is equidistant
  // interpolated, which does not translate into equidistant spline interpolation
  constexpr double eps = 0.15;
  EXPECT_POINT_NEAR(trajectory[0], ans[0], eps);
  EXPECT_POINT_NEAR(trajectory[1], ans[1], eps);
  EXPECT_POINT_NEAR(trajectory[2], ans[2], eps);
  EXPECT_POINT_NEAR(trajectory[3], ans[3], eps);
  EXPECT_POINT_NEAR(trajectory[4], ans[4], eps);
  EXPECT_POINT_NEAR(trajectory[5], ans[5], eps);
  EXPECT_POINT_NEAR(trajectory[6], ans[6], eps);
  EXPECT_POINT_NEAR(trajectory[7], ans[7], eps);
  EXPECT_POINT_NEAR(trajectory[8], ans[8], eps);
  EXPECT_POINT_NEAR(trajectory[9], ans[9], eps);
  EXPECT_POINT_NEAR(trajectory[10], ans[10], eps);
}

TEST(HermiteCurveTest, getPointLine)
{
  const auto curve = makeLine2();

  // The uncertainty of the calculations is high because the parameter space is equidistant
  // interpolated, which does not translate into equidistant spline interpolation
  constexpr double eps = 0.15;
  EXPECT_POINT_NEAR(curve.getPoint(0.0, false), makePoint(0.0, 0.0), eps);
  EXPECT_POINT_NEAR(curve.getPoint(0.1, false), makePoint(0.2, 0.2), eps);
  EXPECT_POINT_NEAR(curve.getPoint(0.2, false), makePoint(0.4, 0.4), eps);
  EXPECT_POINT_NEAR(curve.getPoint(0.3, false), makePoint(0.6, 0.6), eps);
  EXPECT_POINT_NEAR(curve.getPoint(0.4, false), makePoint(0.8, 0.8), eps);
  EXPECT_POINT_NEAR(curve.getPoint(0.5, false), makePoint(1.0, 1.0), eps);
  EXPECT_POINT_NEAR(curve.getPoint(0.6, false), makePoint(1.2, 1.2), eps);
  EXPECT_POINT_NEAR(curve.getPoint(0.7, false), makePoint(1.4, 1.4), eps);
  EXPECT_POINT_NEAR(curve.getPoint(0.8, false), makePoint(1.6, 1.6), eps);
  EXPECT_POINT_NEAR(curve.getPoint(0.9, false), makePoint(1.8, 1.8), eps);
  EXPECT_POINT_NEAR(curve.getPoint(1.0, false), makePoint(2.0, 2.0), eps);
}

TEST(HermiteCurveTest, getPointCurve)
{
  geometry_msgs::msg::Pose start_pose = makePose(0, 0);
  geometry_msgs::msg::Pose goal_pose = makePose(1, 1);
  geometry_msgs::msg::Vector3 start_vec = makeVector(0, 1);
  geometry_msgs::msg::Vector3 goal_vec = makeVector(1, 0);
  math::geometry::HermiteCurve curve(start_pose, goal_pose, start_vec, goal_vec);

  // The uncertainty of the calculations is high because the parameter space is equidistant
  // interpolated, which does not translate into equidistant spline interpolation
  constexpr double eps = 0.15;
  EXPECT_POINT_NEAR(curve.getPoint(0.0, true), makePoint(0.0, 0.0), eps);
  EXPECT_POINT_NEAR(
    curve.getPoint(0.75, true), makePoint(1.0 - 1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0)), eps);
  EXPECT_POINT_NEAR(curve.getPoint(1.5, true), makePoint(1.0, 1.0), eps);
}

TEST(HermiteCurveTest, get2DCurvatureLine)
{
  const auto curve = makeLine1();
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.0, true), 0.0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.1, true), 0.0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.2, true), 0.0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.3, true), 0.0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.4, true), 0.0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.5, true), 0.0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.6, true), 0.0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.7, true), 0.0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.8, true), 0.0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(0.9, true), 0.0);
  EXPECT_DOUBLE_EQ(curve.get2DCurvature(1.0, true), 0.0);
}

TEST(HermiteCurveTest, get2DCurvatureCurve)
{
  constexpr double eps = 0.01;
  const auto curve1 = makeCurve1();
  EXPECT_NEAR(curve1.get2DCurvature(0.0, false), 4.0, eps);
  EXPECT_NEAR(curve1.get2DCurvature(0.5, false), 0.45, eps);
  EXPECT_NEAR(curve1.get2DCurvature(1.0, false), 4.0, eps);

  const auto curve2 = makeCurve1(false);
  EXPECT_NEAR(curve2.get2DCurvature(0.0, false), -4.0, eps);
  EXPECT_NEAR(curve2.get2DCurvature(0.5, false), -0.45, eps);
  EXPECT_NEAR(curve2.get2DCurvature(1.0, false), -4.0, eps);
}

TEST(HermiteCurveTest, getMaximum2DCurvatureLine)
{
  const auto curve = makeLine1();
  EXPECT_DOUBLE_EQ(curve.getMaximum2DCurvature(), 0.0);
}

TEST(HermiteCurveTest, getMaximum2DCurvatureCurve)
{
  constexpr double eps = 0.01;
  const auto curve1 = makeCurve1();
  EXPECT_NEAR(curve1.getMaximum2DCurvature(), 4.0, eps);

  const auto curve2 = makeCurve1(false);
  EXPECT_NEAR(curve2.getMaximum2DCurvature(), -4.0, eps);
}

TEST(HermiteCurveTest, getLengthNoParameter)
{
  const auto curve = makeLine1();
  EXPECT_NEAR(curve.getLength(), 1.0, EPS);
}

TEST(HermiteCurveTest, getLengthParameter)
{
  const auto curve = makeLine1();
  EXPECT_NEAR(curve.getLength(1000), 1.0, EPS);
}

TEST(HermiteCurveTest, getSValue)
{
  const auto curve = makeLine2();

  const auto s0 = curve.getSValue(makePose(0.0, 0.0), 1.0, false);
  EXPECT_TRUE(s0);
  EXPECT_NEAR(s0.value(), 0.0, EPS);
  const auto s1 = curve.getSValue(makePose(1.0, 1.0), 1.0, false);
  EXPECT_TRUE(s1);
  EXPECT_NEAR(s1.value(), 0.5, EPS);
  const auto s2 = curve.getSValue(makePose(2.0, 2.0), 1.0, false);
  EXPECT_TRUE(s2);
  EXPECT_NEAR(s2.value(), 1.0, EPS);
}

/**
 * @note Test function correctness with parameter denormalize_s = true.
 */
TEST(HermiteCurveTest, getSValueDenormalized)
{
  const auto curve = makeLine2();

  const auto s0 = curve.getSValue(makePose(0.0, 0.0), 1.0, true);
  EXPECT_TRUE(s0);
  EXPECT_NEAR(s0.value(), 0.0, EPS);
  const auto s1 = curve.getSValue(makePose(1.0, 1.0), 1.0, true);
  EXPECT_TRUE(s1);
  EXPECT_NEAR(s1.value(), 0.5 * 2.0 * std::sqrt(2.0), EPS);
  const auto s2 = curve.getSValue(makePose(2.0, 2.0), 1.0, true);
  EXPECT_TRUE(s2);
  EXPECT_NEAR(s2.value(), 2.0 * std::sqrt(2.0), EPS);
}

TEST(HermiteCurveTest, getSquaredDistanceIn2D)
{
  const auto curve = makeLine2();

  EXPECT_NEAR(curve.getSquaredDistanceIn2D(makePoint(2.0, 0.0), 0.5, false), 2.0, EPS);
  EXPECT_NEAR(curve.getSquaredDistanceIn2D(makePoint(2.0, 0.0), 1.0, false), 4.0, EPS);
  EXPECT_NEAR(curve.getSquaredDistanceIn2D(makePoint(-1.0, -2.0), 0.0, false), 5.0, EPS);
}

TEST(HermiteCurveTest, getSquaredDistanceIn2DZeroDistance)
{
  const auto curve = makeLine2();

  EXPECT_NEAR(
    curve.getSquaredDistanceIn2D(makePoint(2.0, 2.0), std::sqrt(2.0) * 2.0, true), 0.0, EPS);
  EXPECT_NEAR(curve.getSquaredDistanceIn2D(makePoint(1.0, 1.0), std::sqrt(2.0), true), 0.0, EPS);
  EXPECT_NEAR(curve.getSquaredDistanceIn2D(makePoint(0.0, 0.0), 0.0, true), 0.0, EPS);
}

TEST(HermiteCurveTest, getSquaredDistanceVector)
{
  const auto curve = makeLine2();

  const auto p0 = curve.getSquaredDistanceVector(makePoint(2.0, 0.0), 0.5, false);
  EXPECT_POINT_NEAR(p0, makePoint(1.0, -1.0, 0.0), EPS);
  const auto p1 = curve.getSquaredDistanceVector(makePoint(2.0, 0.0), 1.0, false);
  EXPECT_POINT_NEAR(p1, makePoint(0.0, -2.0, 0.0), EPS);
  const auto p2 = curve.getSquaredDistanceVector(makePoint(-1.0, -2.0), 0.0, false);
  EXPECT_POINT_NEAR(p2, makePoint(-1.0, -2.0, 0.0), EPS);
}

TEST(HermiteCurveTest, getSquaredDistanceVectorZeroDistance)
{
  const auto curve = makeLine2();

  const auto p0 = curve.getSquaredDistanceVector(makePoint(2.0, 2.0), std::sqrt(2.0) * 2.0, true);
  EXPECT_POINT_NEAR(p0, makePoint(0.0, 0.0, 0.0), EPS);
  const auto p1 = curve.getSquaredDistanceVector(makePoint(1.0, 1.0), std::sqrt(2.0), true);
  EXPECT_POINT_NEAR(p1, makePoint(0.0, 0.0, 0.0), EPS);
  const auto p2 = curve.getSquaredDistanceVector(makePoint(0.0, 0.0), 0.0, true);
  EXPECT_POINT_NEAR(p2, makePoint(0.0, 0.0, 0.0), EPS);
}

TEST(HermiteCurveTest, getCollisionPointIn2DLine)
{
  const auto curve = makeLine2();

  constexpr double eps = 0.1;
  const auto s = curve.getCollisionPointIn2D(makePoint(1.0, 0.0), makePoint(0.0, 1.0));
  EXPECT_TRUE(s);
  EXPECT_NEAR(s.value(), 0.25, eps);
}

TEST(HermiteCurveTest, getCollisionPointIn2DLineNoCollision)
{
  const auto curve = makeLine2();

  const auto s = curve.getCollisionPointIn2D(makePoint(1.0, 0.0), makePoint(2.0, 1.0));
  EXPECT_FALSE(s);
}

TEST(HermiteCurveTest, getCollisionPointIn2DCurve)
{
  const auto curve = makeCurve1();

  constexpr double eps = 0.1;
  const auto s0 = curve.getCollisionPointIn2D(makePoint(1.0, 0.0), makePoint(0.0, 1.0));
  EXPECT_TRUE(s0);
  EXPECT_NEAR(s0.value(), 0.5, eps);

  const auto s1 = curve.getCollisionPointIn2D(makePoint(0.1, 0.0), makePoint(1.0, 0.9));
  EXPECT_TRUE(s1);
  EXPECT_NEAR(s1.value(), 0.2, eps);

  const auto s2 = curve.getCollisionPointIn2D(makePoint(0.1, 0.0), makePoint(1.0, 0.9), true);
  EXPECT_TRUE(s2);
  EXPECT_NEAR(s2.value(), 0.8, eps);
}

TEST(HermiteCurveTest, getCollisionPointIn2DCurveEdge)
{
  const auto curve = makeCurve1();

  constexpr double eps = 0.1;
  const auto s = curve.getCollisionPointIn2D(makePoint(-1.0, 1.0), makePoint(1.0, -1.0));
  EXPECT_TRUE(s);
  EXPECT_NEAR(s.value(), 0.0, eps);
}

TEST(HermiteCurveTest, getCollisionPointIn2DVectorWrongCases)
{
  const auto curve = makeCurve1();

  std::vector<geometry_msgs::msg::Point> polygon;
  EXPECT_FALSE(curve.getCollisionPointIn2D(polygon));
  polygon.emplace_back(makePoint(1.0, 1.0));
  EXPECT_FALSE(curve.getCollisionPointIn2D(polygon));
}

TEST(HermiteCurveTest, getCollisionPointIn2DVectorOneCollision)
{
  const auto curve = makeLine1();

  std::vector<geometry_msgs::msg::Point> polygon{
    makePoint(0.5, 0.5), makePoint(-0.5, 0.5), makePoint(-0.5, -0.5), makePoint(0.5, -0.5)};
  const auto s = curve.getCollisionPointIn2D(polygon);
  EXPECT_TRUE(s);
  EXPECT_NEAR(s.value(), 0.5, EPS);
  EXPECT_FALSE(curve.getCollisionPointIn2D(polygon, false, false));
}

TEST(HermiteCurveTest, getCollisionPointIn2DVectorMultipleCollisions)
{
  const auto curve = makeLine2();

  std::vector<geometry_msgs::msg::Point> polygon{
    makePoint(1, 0), makePoint(2, 1), makePoint(1, 2), makePoint(0, 1)};

  constexpr double eps = 0.1;
  const auto s0 = curve.getCollisionPointIn2D(polygon);
  EXPECT_TRUE(s0);
  EXPECT_NEAR(s0.value(), 0.25, eps);

  const auto s1 = curve.getCollisionPointIn2D(polygon, true);
  EXPECT_TRUE(s1);
  EXPECT_NEAR(s1.value(), 0.75, eps);

  const auto s2 = curve.getCollisionPointIn2D(polygon, false, false);
  EXPECT_TRUE(s2);
  EXPECT_NEAR(s2.value(), 0.75, eps);
}

TEST(HermiteCurveTest, getTangentVector1)
{  //p(0,0) v(1,0)-> p(1,1) v(0,1)
  const auto curve = makeCurve1();
  double norm =
    std::hypot(curve.getTangentVector(0.5, false).x, curve.getTangentVector(0.5, false).y);
  EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x / norm, 1.0 / std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).y / norm, 1.0 / std::sqrt(2.0));
}

TEST(HermiteCurveTest, getTangentVector2)
{  //p(0,0) v(1,0)-> p(1,-1) v(0,-1)
  const auto curve = makeCurve2();
  double norm =
    std::hypot(curve.getTangentVector(0.5, false).x, curve.getTangentVector(0.5, false).y);
  EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x / norm, 1.0 / std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).y / norm, -1.0 / std::sqrt(2.0));
}

TEST(HermiteCurveTest, getTangentVector3)
{  //p(1,1) v(0,-1)-> p(0,0) v(-1,0)
  const auto curve = makeCurve3();
  double norm =
    std::hypot(curve.getTangentVector(0.5, false).x, curve.getTangentVector(0.5, false).y);
  EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x / norm, -1.0 / std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).y / norm, -1.0 / std::sqrt(2.0));
}

TEST(HermiteCurveTest, getTangentVector4)
{  //p(1,-1) v(0,1)-> p(0,0) v(-1,0)
  const auto curve = makeCurve4();
  double norm =
    std::hypot(curve.getTangentVector(0.5, false).x, curve.getTangentVector(0.5, false).y);
  EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).x / norm, -1.0 / std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(curve.getTangentVector(0.5, false).y / norm, 1.0 / std::sqrt(2.0));
}

/**
 * @note Test function correctness with parameter denormalize_s = true with different curves.
 */
TEST(HermiteCurveTest, getTangentVectorDenormalized1)
{  //p(0,0) v(1,0)-> p(1,1) v(0,1)
  const auto curve = makeCurve1();
  constexpr double eps = 0.1;
  double norm =
    std::hypot(curve.getTangentVector(0.75, true).x, curve.getTangentVector(0.75, true).y);
  EXPECT_NEAR(curve.getTangentVector(0.75, true).x / norm, 1.0 / std::sqrt(2.0), eps);
  EXPECT_NEAR(curve.getTangentVector(0.75, true).y / norm, 1.0 / std::sqrt(2.0), eps);
}

/**
 * @note Test function correctness with parameter denormalize_s = true with different curves.
 */
TEST(HermiteCurveTest, getTangentVectorDenormalized2)
{  //p(0,0) v(1,0)-> p(1,-1) v(0,-1)
  const auto curve = makeCurve2();
  constexpr double eps = 0.1;
  double norm =
    std::hypot(curve.getTangentVector(0.75, true).x, curve.getTangentVector(0.75, true).y);
  EXPECT_NEAR(curve.getTangentVector(0.75, true).x / norm, 1.0 / std::sqrt(2.0), eps);
  EXPECT_NEAR(curve.getTangentVector(0.75, true).y / norm, -1.0 / std::sqrt(2.0), eps);
}

/**
 * @note Test function correctness with parameter denormalize_s = true with different curves.
 */
TEST(HermiteCurveTest, getTangentVectorDenormalized3)
{  //p(1,1) v(0,-1)-> p(0,0) v(-1,0)
  const auto curve = makeCurve3();
  constexpr double eps = 0.1;
  double norm =
    std::hypot(curve.getTangentVector(0.75, true).x, curve.getTangentVector(0.75, true).y);
  EXPECT_NEAR(curve.getTangentVector(0.75, true).x / norm, -1.0 / std::sqrt(2.0), eps);
  EXPECT_NEAR(curve.getTangentVector(0.75, true).y / norm, -1.0 / std::sqrt(2.0), eps);
}

/**
 * @note Test function correctness with parameter denormalize_s = true with different curves.
 */
TEST(HermiteCurveTest, getTangentVectorDenormalized4)
{  //p(1,-1) v(0,1)-> p(0,0) v(-1,0)
  const auto curve = makeCurve4();
  constexpr double eps = 0.1;
  double norm =
    std::hypot(curve.getTangentVector(0.75, true).x, curve.getTangentVector(0.75, true).y);
  EXPECT_NEAR(curve.getTangentVector(0.75, true).x / norm, -1.0 / std::sqrt(2.0), eps);
  EXPECT_NEAR(curve.getTangentVector(0.75, true).y / norm, 1.0 / std::sqrt(2.0), eps);
}

TEST(HermiteCurveTest, getNormalVector1)
{  //p(0,0) v(1,0)-> p(1,1) v(0,1)
  const auto curve = makeCurve1();
  double norm =
    std::hypot(curve.getNormalVector(0.5, false).x, curve.getNormalVector(0.5, false).y);
  EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).x / norm, -1.0 / std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).y / norm, 1.0 / std::sqrt(2.0));
}

TEST(HermiteCurveTest, getNormalVector2)
{  //p(0,0) v(1,0)-> p(1,-1) v(0,-1)
  const auto curve = makeCurve2();
  double norm =
    std::hypot(curve.getNormalVector(0.5, false).x, curve.getNormalVector(0.5, false).y);
  EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).x / norm, 1.0 / std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).y / norm, 1.0 / std::sqrt(2.0));
}

TEST(HermiteCurveTest, getNormalVector3)
{  //p(1,1) v(0,-1)-> p(0,0) v(-1,0)
  const auto curve = makeCurve3();
  double norm =
    std::hypot(curve.getNormalVector(0.5, false).x, curve.getNormalVector(0.5, false).y);
  EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).x / norm, 1.0 / std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).y / norm, -1.0 / std::sqrt(2.0));
}

TEST(HermiteCurveTest, getNormalVector4)
{  //p(1,-1) v(0,1)-> p(0,0) v(-1,0)
  const auto curve = makeCurve4();
  double norm =
    std::hypot(curve.getNormalVector(0.5, false).x, curve.getNormalVector(0.5, false).y);
  EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).x / norm, -1.0 / std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(curve.getNormalVector(0.5, false).y / norm, -1.0 / std::sqrt(2.0));
}

/**
 * @note Test function correctness with parameter denormalize_s = true with different curves.
 */
TEST(HermiteCurveTest, getNormalVectorDenormalized1)
{  //p(0,0) v(1,0)-> p(1,1) v(0,1)
  const auto curve = makeCurve1();
  constexpr double eps = 0.1;
  double norm =
    std::hypot(curve.getNormalVector(0.75, true).x, curve.getNormalVector(0.75, true).y);
  EXPECT_NEAR(curve.getNormalVector(0.75, true).x / norm, -1.0 / std::sqrt(2.0), eps);
  EXPECT_NEAR(curve.getNormalVector(0.75, true).y / norm, 1.0 / std::sqrt(2.0), eps);
}

/**
 * @note Test function correctness with parameter denormalize_s = true with different curves.
 */
TEST(HermiteCurveTest, getNormalVectorDenormalized2)
{  //p(0,0) v(1,0)-> p(1,-1) v(0,-1)
  const auto curve = makeCurve2();
  constexpr double eps = 0.1;
  double norm =
    std::hypot(curve.getNormalVector(0.75, true).x, curve.getNormalVector(0.75, true).y);
  EXPECT_NEAR(curve.getNormalVector(0.75, true).x / norm, 1.0 / std::sqrt(2.0), eps);
  EXPECT_NEAR(curve.getNormalVector(0.75, true).y / norm, 1.0 / std::sqrt(2.0), eps);
}

/**
 * @note Test function correctness with parameter denormalize_s = true with different curves.
 */
TEST(HermiteCurveTest, getNormalVectorDenormalized3)
{  //p(1,1) v(0,-1)-> p(0,0) v(-1,0)
  const auto curve = makeCurve3();
  constexpr double eps = 0.1;
  double norm =
    std::hypot(curve.getNormalVector(0.75, true).x, curve.getNormalVector(0.75, true).y);
  EXPECT_NEAR(curve.getNormalVector(0.75, true).x / norm, 1.0 / std::sqrt(2.0), eps);
  EXPECT_NEAR(curve.getNormalVector(0.75, true).y / norm, -1.0 / std::sqrt(2.0), eps);
}

/**
 * @note Test function correctness with parameter denormalize_s = true with different curves.
 */
TEST(HermiteCurveTest, getNormalVectorDenormalized4)
{  //p(1,-1) v(0,1)-> p(0,0) v(-1,0)
  const auto curve = makeCurve4();
  constexpr double eps = 0.1;
  double norm =
    std::hypot(curve.getNormalVector(0.75, true).x, curve.getNormalVector(0.75, true).y);
  EXPECT_NEAR(curve.getNormalVector(0.75, true).x / norm, -1.0 / std::sqrt(2.0), eps);
  EXPECT_NEAR(curve.getNormalVector(0.75, true).y / norm, -1.0 / std::sqrt(2.0), eps);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
