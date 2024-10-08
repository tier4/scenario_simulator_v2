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

#include <cmath>
#include <geometry/intersection/intersection.hpp>
#include <scenario_simulator_exception/exception.hpp>

#include "../expect_eq_macros.hpp"
#include "../test_utils.hpp"

TEST(Intersection, isIntersect2DDisjoint)
{
  math::geometry::LineSegment line0(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  math::geometry::LineSegment line1(makePoint(1.0, 0.0), makePoint(2.0, 1.0));
  EXPECT_FALSE(math::geometry::isIntersect2D(line0, line1));
}

TEST(Intersection, isIntersect2DDisjointVector)
{
  std::vector<math::geometry::LineSegment> lines{
    {makePoint(0.0, 0.0), makePoint(1.0, 1.0)}, {makePoint(1.0, 0.0), makePoint(2.0, 1.0)}};
  EXPECT_FALSE(math::geometry::isIntersect2D(lines));
}

TEST(Intersection, isIntersect2DIntersect)
{
  math::geometry::LineSegment line0(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  math::geometry::LineSegment line1(makePoint(1.0, 0.0), makePoint(0.0, 1.0));
  EXPECT_TRUE(math::geometry::isIntersect2D(line0, line1));
}

TEST(Intersection, isIntersect2DIntersectVector)
{
  std::vector<math::geometry::LineSegment> lines{
    {makePoint(1.0, 0.0), makePoint(0.0, 1.0)}, {makePoint(0.0, 0.0), makePoint(1.0, 1.0)}};
  EXPECT_TRUE(math::geometry::isIntersect2D(lines));
}

TEST(Intersection, isIntersect2DIdentical)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  EXPECT_TRUE(math::geometry::isIntersect2D(line, line));
}

TEST(Intersection, isIntersect2DIdenticalVector)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  std::vector<math::geometry::LineSegment> lines;
  lines.push_back(line);
  lines.push_back(line);
  lines.push_back(line);
  EXPECT_TRUE(math::geometry::isIntersect2D(lines));
}

TEST(Intersection, isIntersect2DEmptyVector)
{
  std::vector<math::geometry::LineSegment> lines;
  bool ans = true;
  EXPECT_NO_THROW(ans = math::geometry::isIntersect2D(lines));
  EXPECT_FALSE(ans);
}

TEST(Intersection, getIntersection2DDisjoint)
{
  math::geometry::LineSegment line0(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  math::geometry::LineSegment line1(makePoint(1.0, 0.0), makePoint(2.0, 1.0));
  EXPECT_FALSE(math::geometry::getIntersection2D(line0, line1));
}

TEST(Intersection, getIntersection2DDisjointVector)
{
  std::vector<math::geometry::LineSegment> lines{
    {makePoint(0.0, 0.0), makePoint(1.0, 1.0)}, {makePoint(1.0, 0.0), makePoint(2.0, 1.0)}};
  EXPECT_EQ(math::geometry::getIntersection2D(lines).size(), size_t(0));
}

TEST(Intersection, getIntersection2DIntersect)
{
  math::geometry::LineSegment line0(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  math::geometry::LineSegment line1(makePoint(1.0, 0.0), makePoint(0.0, 1.0));
  auto ans = math::geometry::getIntersection2D(line0, line1);
  EXPECT_TRUE(ans);
  EXPECT_POINT_EQ(ans.value(), makePoint(0.5, 0.5));
}

TEST(Intersection, getIntersection2DIntersectVector)
{
  std::vector<math::geometry::LineSegment> lines{
    {makePoint(0.0, 0.0), makePoint(1.0, 1.0)}, {makePoint(1.0, 0.0), makePoint(0.0, 1.0)}};
  auto ans = math::geometry::getIntersection2D(lines);
  EXPECT_EQ(ans.size(), size_t(2));
  EXPECT_POINT_EQ(ans[0], makePoint(0.5, 0.5));
  EXPECT_POINT_EQ(ans[1], makePoint(0.5, 0.5));
}

TEST(Intersection, getIntersection2DIdentical)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  EXPECT_THROW(math::geometry::getIntersection2D(line, line), common::SimulationError);
}

TEST(Intersection, getIntersection2DIdenticalVector)
{
  math::geometry::LineSegment line(makePoint(0, 0), makePoint(1, 1));
  std::vector<math::geometry::LineSegment> lines;
  lines.push_back(line);
  lines.push_back(line);
  lines.push_back(line);

  EXPECT_THROW(math::geometry::getIntersection2D(lines), common::SimulationError);
}

TEST(Intersection, getIntersection2DEmptyVector)
{
  std::vector<math::geometry::LineSegment> lines;
  std::vector<geometry_msgs::msg::Point> ans;
  EXPECT_NO_THROW(ans = math::geometry::getIntersection2D(lines));
  EXPECT_EQ(ans.size(), size_t(0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
