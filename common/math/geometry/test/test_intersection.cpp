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

#include "expect_eq_macros.hpp"

geometry_msgs::msg::Point makePoint(double x, double y, double z = 0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::msg::Vector3 makeVector(double x, double y, double z = 0)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

TEST(Intersection, isIntersect2DDisjoint)
{
  math::geometry::LineSegment line0(makePoint(0, 0), makePoint(1, 1));
  math::geometry::LineSegment line1(makePoint(1, 0), makePoint(2, 1));
  EXPECT_FALSE(math::geometry::isIntersect2D(line0, line1));
}

TEST(Intersection, isIntersect2DDisjointVector)
{
  std::vector<math::geometry::LineSegment> lines;
  lines.emplace_back(makePoint(0, 0), makePoint(1, 1));
  lines.emplace_back(makePoint(1, 0), makePoint(2, 1));
  EXPECT_FALSE(math::geometry::isIntersect2D(lines));
}

TEST(Intersection, isIntersect2DIntersect)
{
  math::geometry::LineSegment line0(makePoint(0, 0), makePoint(1, 1));
  math::geometry::LineSegment line1(makePoint(1, 0), makePoint(0, 1));
  EXPECT_TRUE(math::geometry::isIntersect2D(line0, line1));
}

TEST(Intersection, isIntersect2DIntersectVector)
{
  std::vector<math::geometry::LineSegment> lines;
  lines.emplace_back(makePoint(0, 0), makePoint(1, 1));
  lines.emplace_back(makePoint(1, 0), makePoint(0, 1));
  EXPECT_TRUE(math::geometry::isIntersect2D(lines));
}

TEST(Intersection, isIntersect2DIdentical)
{
  math::geometry::LineSegment line(makePoint(0, 0), makePoint(1, 1));
  EXPECT_TRUE(math::geometry::isIntersect2D(line, line));
}

TEST(Intersection, isIntersect2DIdenticalVector)
{
  math::geometry::LineSegment line(makePoint(0, 0), makePoint(1, 1));
  std::vector<math::geometry::LineSegment> lines;
  lines.emplace_back(line);
  lines.emplace_back(line);
  lines.emplace_back(line);
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
  math::geometry::LineSegment line0(makePoint(0, 0), makePoint(1, 1));
  math::geometry::LineSegment line1(makePoint(1, 0), makePoint(2, 1));
  EXPECT_FALSE(math::geometry::getIntersection2D(line0, line1));
}

TEST(Intersection, getIntersection2DDisjointVector)
{
  std::vector<math::geometry::LineSegment> lines;
  lines.emplace_back(makePoint(0, 0), makePoint(1, 1));
  lines.emplace_back(makePoint(1, 0), makePoint(2, 1));
  EXPECT_TRUE(math::geometry::getIntersection2D(lines).empty());
}

TEST(Intersection, getIntersection2DIntersect)
{
  math::geometry::LineSegment line0(makePoint(0, 0), makePoint(1, 1));
  math::geometry::LineSegment line1(makePoint(1, 0), makePoint(0, 1));
  auto ans = math::geometry::getIntersection2D(line0, line1);
  EXPECT_TRUE(ans);
  EXPECT_POINT_EQ(ans.value(), makePoint(0.5, 0.5));
}

TEST(Intersection, getIntersection2DIntersectVector)
{
  std::vector<math::geometry::LineSegment> lines;
  lines.emplace_back(makePoint(0, 0), makePoint(1, 1));
  lines.emplace_back(makePoint(1, 0), makePoint(0, 1));
  auto ans = math::geometry::getIntersection2D(lines);
  EXPECT_EQ(ans.size(), size_t(2));
  EXPECT_POINT_EQ(ans[0], makePoint(0.5, 0.5));
  EXPECT_POINT_EQ(ans[1], makePoint(0.5, 0.5));
}

TEST(Intersection, getIntersection2DIdentical)
{
  math::geometry::LineSegment line(makePoint(0, 0), makePoint(1, 1));
  auto ans = math::geometry::getIntersection2D(line, line);
  EXPECT_TRUE(ans);
  EXPECT_TRUE(std::isnan(ans.value().x));
  EXPECT_TRUE(std::isnan(ans.value().y));
  EXPECT_TRUE(std::isnan(ans.value().z));
}

TEST(Intersection, getIntersection2DIdenticalVector)
{
  math::geometry::LineSegment line(makePoint(0, 0), makePoint(1, 1));
  std::vector<math::geometry::LineSegment> lines;
  lines.emplace_back(line);
  lines.emplace_back(line);
  lines.emplace_back(line);

  auto ans = math::geometry::getIntersection2D(lines);
  EXPECT_EQ(ans.size(), size_t(6));
  // point 0
  EXPECT_TRUE(std::isnan(ans[0].x));
  EXPECT_TRUE(std::isnan(ans[0].y));
  EXPECT_TRUE(std::isnan(ans[0].z));
  // point 1
  EXPECT_TRUE(std::isnan(ans[1].x));
  EXPECT_TRUE(std::isnan(ans[1].y));
  EXPECT_TRUE(std::isnan(ans[1].z));
  // point 2
  EXPECT_TRUE(std::isnan(ans[2].x));
  EXPECT_TRUE(std::isnan(ans[2].y));
  EXPECT_TRUE(std::isnan(ans[2].z));
  // point 3
  EXPECT_TRUE(std::isnan(ans[3].x));
  EXPECT_TRUE(std::isnan(ans[3].y));
  EXPECT_TRUE(std::isnan(ans[3].z));
  // point 4
  EXPECT_TRUE(std::isnan(ans[4].x));
  EXPECT_TRUE(std::isnan(ans[4].y));
  EXPECT_TRUE(std::isnan(ans[4].z));
  // point 5
  EXPECT_TRUE(std::isnan(ans[5].x));
  EXPECT_TRUE(std::isnan(ans[5].y));
  EXPECT_TRUE(std::isnan(ans[5].z));
}

TEST(Intersection, getIntersection2DEmptyVector)
{
  std::vector<math::geometry::LineSegment> lines;
  std::vector<geometry_msgs::msg::Point> ans;
  EXPECT_NO_THROW(ans = math::geometry::getIntersection2D(lines));
  EXPECT_TRUE(ans.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
