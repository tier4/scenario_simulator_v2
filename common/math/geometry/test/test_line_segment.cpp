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
#include <geometry/polygon/line_segment.hpp>
#include <scenario_simulator_exception/exception.hpp>

#include "expect_eq_macros.hpp"

geometry_msgs::msg::Point makePoint(double x, double y, double z = 0)
{
  geometry_msgs::msg::Point v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

geometry_msgs::msg::Vector3 makeVector(double x, double y, double z = 0)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

TEST(LineSegment, initializeDifferentPoints)
{
  geometry_msgs::msg::Point point0 = makePoint(0, 0);
  geometry_msgs::msg::Point point1 = makePoint(1, 1);
  EXPECT_NO_THROW(const math::geometry::LineSegment line_segment(point0, point1));
  const math::geometry::LineSegment line_segment(point0, point1);
  EXPECT_POINT_EQ(line_segment.start_point, point0);
  EXPECT_POINT_EQ(line_segment.end_point, point1);
}

TEST(LineSegment, initializeIdenticalPoints)
{
  geometry_msgs::msg::Point point = makePoint(0, 0);
  EXPECT_NO_THROW(const math::geometry::LineSegment line_segment(point, point));
  const math::geometry::LineSegment line_segment(point, point);
  EXPECT_POINT_EQ(line_segment.start_point, point);
  EXPECT_POINT_EQ(line_segment.end_point, point);
}

TEST(LineSegment, initializeVector)
{
  geometry_msgs::msg::Point point = makePoint(0, 0);
  geometry_msgs::msg::Vector3 vec = makeVector(1, 1);
  EXPECT_NO_THROW(const math::geometry::LineSegment line_segment(point, vec, 1));
  const math::geometry::LineSegment line_segment(point, vec, 1);
  EXPECT_POINT_EQ(line_segment.start_point, point);
  EXPECT_POINT_EQ(line_segment.end_point, makePoint(std::sqrt(2) / 2, std::sqrt(2) / 2));
}

TEST(LineSegment, initializeVectorZero)
{
  geometry_msgs::msg::Point point = makePoint(0, 0);
  geometry_msgs::msg::Vector3 vec = makeVector(0, 0);
  EXPECT_THROW(
    const math::geometry::LineSegment line_segment(point, vec, 1), common::SimulationError);
}

TEST(LineSegment, initializeVectorZeroLength)
{
  geometry_msgs::msg::Point point = makePoint(0, 0);
  geometry_msgs::msg::Vector3 vec = makeVector(1, 1);
  EXPECT_NO_THROW(const math::geometry::LineSegment line_segment(point, vec, 0));
  const math::geometry::LineSegment line_segment(point, vec, 0);
  EXPECT_POINT_EQ(line_segment.start_point, point);
  EXPECT_POINT_EQ(line_segment.end_point, point);
}

TEST(LineSegment, isIntersect2DDisjoint)
{
  const math::geometry::LineSegment line0(makePoint(0, 0), makePoint(1, 1));
  const math::geometry::LineSegment line1(makePoint(1, 0), makePoint(2, 1));
  EXPECT_FALSE(line0.isIntersect2D(line1));
}

TEST(LineSegment, isIntersect2DIntersect)
{
  const math::geometry::LineSegment line0(makePoint(1, 1), makePoint(2, 1));
  const math::geometry::LineSegment line1(makePoint(1, 0), makeVector(1, 1), 3);
  EXPECT_TRUE(line0.isIntersect2D(line1));
}

TEST(LineSegment, isIntersect2DIdentical)
{
  const math::geometry::LineSegment line(makePoint(0, 0), makePoint(1, 1));
  EXPECT_TRUE(line.isIntersect2D(line));
}

TEST(LineSegment, getIntersection2DDisjoint)
{
  const math::geometry::LineSegment line0(makePoint(0, 0), makePoint(1, 1));
  const math::geometry::LineSegment line1(makePoint(1, 0), makePoint(2, 1));
  EXPECT_FALSE(line0.getIntersection2D(line1));
}

TEST(LineSegment, getIntersection2DIntersect)
{
  const math::geometry::LineSegment line0(makePoint(0, 0), makePoint(1, 1));
  const math::geometry::LineSegment line1(makePoint(1, 0), makePoint(0, 1));
  auto ans = line0.getIntersection2D(line1);
  EXPECT_TRUE(ans);
  EXPECT_POINT_EQ(ans.value(), makePoint(0.5, 0.5));
}

TEST(LineSegment, getIntersection2DIdentical)
{
  const math::geometry::LineSegment line(makePoint(0, 0), makePoint(1, 1));
  auto ans = line.getIntersection2D(line);
  EXPECT_TRUE(ans);
  EXPECT_TRUE(std::isnan(ans.value().x));
  EXPECT_TRUE(std::isnan(ans.value().y));
}

TEST(LineSegment, getVector)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(2, 3, 4));
  EXPECT_POINT_EQ(line.getVector(), makeVector(1, 1, 1));
}

TEST(LineSegment, getVectorZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(1, 2, 3));
  EXPECT_POINT_EQ(line.getVector(), makeVector(0, 0, 0));
}

TEST(LineSegment, get2DVector)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(2, 3, 4));
  EXPECT_POINT_EQ(line.get2DVector(), makeVector(1, 1, 0));
}

TEST(LineSegment, get2DVectorZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(1, 2, 3));
  EXPECT_POINT_EQ(line.get2DVector(), makeVector(0, 0, 0));
}

TEST(LineSegment, getLength)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(2, 3, 4));
  EXPECT_DOUBLE_EQ(line.getLength(), std::sqrt(3));
}

TEST(LineSegment, getLengthZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(1, 2, 3));
  EXPECT_DOUBLE_EQ(line.getLength(), 0.0);
}

TEST(LineSegment, get2DLength)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(2, 3, 4));
  EXPECT_DOUBLE_EQ(line.get2DLength(), std::sqrt(2));
}

TEST(LineSegment, get2DLengthZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(1, 2, 3));
  EXPECT_DOUBLE_EQ(line.get2DLength(), 0.0);
}

TEST(LineSegment, getSlope)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(3, 3, 4));
  EXPECT_DOUBLE_EQ(line.getSlope(), 0.5);
}

TEST(LineSegment, getSlopeZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(1, 2, 3));
  EXPECT_TRUE(std::isnan(line.getSlope()));
}

TEST(LineSegment, getIntercept)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(3, 1, 4));
  EXPECT_DOUBLE_EQ(line.getIntercept(), 2.5);
}

TEST(LineSegment, getInterceptZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1, 2, 3), makePoint(1, 2, 3));
  EXPECT_TRUE(std::isnan(line.getSlope()));
}

TEST(LineSegment, getLineSegments)
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(1, 2, 3), makePoint(2, 3, 4), makePoint(3, 4, 5), makePoint(4, 5, 6)};
  const std::vector<math::geometry::LineSegment> lines = math::geometry::getLineSegments(points);
  EXPECT_EQ(lines.size(), size_t(4));
  EXPECT_POINT_EQ(lines[0].start_point, points[0]);
  EXPECT_POINT_EQ(lines[0].end_point, points[1]);
  EXPECT_POINT_EQ(lines[1].start_point, points[1]);
  EXPECT_POINT_EQ(lines[1].end_point, points[2]);
  EXPECT_POINT_EQ(lines[2].start_point, points[2]);
  EXPECT_POINT_EQ(lines[2].end_point, points[3]);
  EXPECT_POINT_EQ(lines[3].start_point, points[3]);
  EXPECT_POINT_EQ(lines[3].end_point, points[0]);
}

TEST(LineSegment, getLineSegmentsVectorEmpty)
{
  const std::vector<geometry_msgs::msg::Point> points;
  const std::vector<math::geometry::LineSegment> lines = math::geometry::getLineSegments(points);
  EXPECT_TRUE(lines.empty());
}

TEST(LineSegment, getLineSegmentsVectorIdentical)
{
  geometry_msgs::msg::Point point = makePoint(1, 2, 3);
  const std::vector<geometry_msgs::msg::Point> points{point, point, point, point};
  const std::vector<math::geometry::LineSegment> lines = math::geometry::getLineSegments(points);
  EXPECT_EQ(lines.size(), size_t(4));
  EXPECT_POINT_EQ(lines[0].start_point, point);
  EXPECT_POINT_EQ(lines[0].end_point, point);
  EXPECT_POINT_EQ(lines[1].start_point, point);
  EXPECT_POINT_EQ(lines[1].end_point, point);
  EXPECT_POINT_EQ(lines[2].start_point, point);
  EXPECT_POINT_EQ(lines[2].end_point, point);
  EXPECT_POINT_EQ(lines[3].start_point, point);
  EXPECT_POINT_EQ(lines[3].end_point, point);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
