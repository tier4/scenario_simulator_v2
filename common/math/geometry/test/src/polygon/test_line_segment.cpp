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
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <scenario_simulator_exception/exception.hpp>

#include "../expect_eq_macros.hpp"
#include "../test_utils.hpp"

TEST(LineSegment, initializeDifferentPoints)
{
  geometry_msgs::msg::Point point0 = makePoint(0.0, 0.0);
  geometry_msgs::msg::Point point1 = makePoint(1.0, 1.0);
  EXPECT_NO_THROW(const math::geometry::LineSegment line_segment(point0, point1));
  const math::geometry::LineSegment line_segment(point0, point1);
  EXPECT_POINT_EQ(line_segment.start_point, point0);
  EXPECT_POINT_EQ(line_segment.end_point, point1);
}

TEST(LineSegment, initializeIdenticalPoints)
{
  geometry_msgs::msg::Point point = makePoint(0.0, 0.0);
  EXPECT_NO_THROW(const math::geometry::LineSegment line_segment(point, point));
  const math::geometry::LineSegment line_segment(point, point);
  EXPECT_POINT_EQ(line_segment.start_point, point);
  EXPECT_POINT_EQ(line_segment.end_point, point);
}

TEST(LineSegment, initializeVector)
{
  geometry_msgs::msg::Point point = makePoint(0.0, 0.0);
  geometry_msgs::msg::Vector3 vec = makeVector(1.0, 1.0);
  EXPECT_NO_THROW(const math::geometry::LineSegment line_segment(point, vec, 1.0));
  const math::geometry::LineSegment line_segment(point, vec, 1.0);
  EXPECT_POINT_EQ(line_segment.start_point, point);
  EXPECT_POINT_EQ(line_segment.end_point, makePoint(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0));
}

TEST(LineSegment, initializeVectorZero)
{
  geometry_msgs::msg::Point point = makePoint(0.0, 0.0);
  geometry_msgs::msg::Vector3 vec = makeVector(0.0, 0.0);
  EXPECT_THROW(
    const math::geometry::LineSegment line_segment(point, vec, 1.0), common::SimulationError);
}

TEST(LineSegment, initializeVectorZeroLength)
{
  geometry_msgs::msg::Point point = makePoint(0.0, 0.0);
  geometry_msgs::msg::Vector3 vec = makeVector(1.0, 1.0);
  EXPECT_NO_THROW(const math::geometry::LineSegment line_segment(point, vec, 0.0));
  const math::geometry::LineSegment line_segment(point, vec, 0.0);
  EXPECT_POINT_EQ(line_segment.start_point, point);
  EXPECT_POINT_EQ(line_segment.end_point, point);
}

TEST(LineSegment, isIntersect2DDisjoint)
{
  const math::geometry::LineSegment line0(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  const math::geometry::LineSegment line1(makePoint(1.0, 0.0), makePoint(2.0, 1.0));
  EXPECT_FALSE(line0.isIntersect2D(line1));
}

TEST(LineSegment, isIntersect2DIntersect)
{
  const math::geometry::LineSegment line0(makePoint(1.0, 1.0), makePoint(2.0, 1.0));
  const math::geometry::LineSegment line1(makePoint(1.0, 0.0), makeVector(1.0, 1.0), 3.0);
  EXPECT_TRUE(line0.isIntersect2D(line1));
}

TEST(LineSegment, isIntersect2DIdentical)
{
  const math::geometry::LineSegment line(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  EXPECT_TRUE(line.isIntersect2D(line));
}

TEST(LineSegment, getIntersection2DDisjoint)
{
  const math::geometry::LineSegment line0(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  const math::geometry::LineSegment line1(makePoint(1.0, 0.0), makePoint(2.0, 1.0));
  EXPECT_FALSE(line0.getIntersection2D(line1));
}

TEST(LineSegment, getIntersection2DIntersect)
{
  const auto line0 = math::geometry::LineSegment(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  const auto line1 = math::geometry::LineSegment(makePoint(1.0, 0.0), makePoint(0.0, 1.0));

  const auto p0 = line0.getIntersection2D(line1);
  const auto p1 = line1.getIntersection2D(line0);

  ASSERT_TRUE(p0.has_value());
  ASSERT_TRUE(p1.has_value());
  EXPECT_POINT_EQ(p0.value(), makePoint(0.5, 0.5));
  EXPECT_POINT_EQ(p1.value(), makePoint(0.5, 0.5));
}

TEST(LineSegment, getIntersection2DIdentical)
{
  const auto line = math::geometry::LineSegment(makePoint(0.0, 0.0), makePoint(1.0, 1.0));

  EXPECT_THROW(line.getIntersection2D(line), common::SimulationError);
}

TEST(LineSegment, getVector)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0));
  EXPECT_VECTOR3_EQ(line.vector, makeVector(1.0, 1.0, 1.0));
}

TEST(LineSegment, getVectorZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_VECTOR3_EQ(line.vector, makeVector(0.0, 0.0, 0.0));
}

TEST(LineSegment, getNormalVector)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0));
  EXPECT_VECTOR3_EQ(line.getNormalVector(), makeVector(-1.0, 1.0, 0.0));
}

TEST(LineSegment, getNormalVector_zeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_VECTOR3_EQ(line.getNormalVector(), makeVector(0.0, 0.0, 0.0));
}

TEST(LineSegment, get2DVector)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0));
  EXPECT_VECTOR3_EQ(line.vector_2d, makeVector(1.0, 1.0, 0.0));
}

TEST(LineSegment, get2DVectorZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_VECTOR3_EQ(line.vector_2d, makeVector(0.0, 0.0, 0.0));
}

TEST(LineSegment, getLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0));
  EXPECT_DOUBLE_EQ(line.length, std::sqrt(3.0));
}

TEST(LineSegment, getLengthZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_DOUBLE_EQ(line.length, 0.0);
}

TEST(LineSegment, get2DLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0));
  EXPECT_DOUBLE_EQ(line.length_2d, std::sqrt(2.0));
}

TEST(LineSegment, get2DLengthZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_DOUBLE_EQ(line.length_2d, 0.0);
}

TEST(LineSegment, get2DVectorSlope)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(3.0, 3.0, 4.0));
  EXPECT_DOUBLE_EQ(line.get2DVectorSlope(), 0.5);
}

TEST(LineSegment, get2DVectorSlopeZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_THROW(line.get2DVectorSlope(), common::SimulationError);
}

TEST(LineSegment, getSquaredDistanceIn2D)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(3.0, 4.0, 5.0));
  EXPECT_DOUBLE_EQ(line.getSquaredDistanceIn2D(makePoint(0.0, 1.0, 2.0), 0.5, false), 8.0);
}

TEST(LineSegment, getSquaredDistanceIn2D_denormalize)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(3.0, 4.0, 5.0));
  EXPECT_DOUBLE_EQ(
    line.getSquaredDistanceIn2D(makePoint(0.0, 1.0, 2.0), std::sqrt(3.0), true), 8.0);
}

TEST(LineSegment, getSquaredDistanceVector)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(3.0, 4.0, 5.0));
  EXPECT_VECTOR3_EQ(
    line.getSquaredDistanceVector(makePoint(0.0, 1.0, 2.0), 0.5, false),
    makeVector(-2.0, -2.0, -2.0));
}

TEST(LineSegment, getSquaredDistanceVector_denormalize)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(3.0, 4.0, 5.0));
  EXPECT_VECTOR3_EQ(
    line.getSquaredDistanceVector(makePoint(0.0, 1.0, 2.0), std::sqrt(3.0), true),
    makeVector(-2.0, -2.0, -2.0));
}

TEST(LineSegment, getLineSegments)
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0), makePoint(3.0, 4.0, 5.0),
    makePoint(4.0, 5.0, 6.0)};
  const std::vector<math::geometry::LineSegment> lines =
    math::geometry::getLineSegments(points, false);
  EXPECT_EQ(lines.size(), size_t(3));
  EXPECT_POINT_EQ(lines[0].start_point, points[0]);
  EXPECT_POINT_EQ(lines[0].end_point, points[1]);
  EXPECT_POINT_EQ(lines[1].start_point, points[1]);
  EXPECT_POINT_EQ(lines[1].end_point, points[2]);
  EXPECT_POINT_EQ(lines[2].start_point, points[2]);
  EXPECT_POINT_EQ(lines[2].end_point, points[3]);
}

TEST(LineSegment, getLineSegments_closeStartEnd)
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0), makePoint(3.0, 4.0, 5.0),
    makePoint(4.0, 5.0, 6.0)};
  const std::vector<math::geometry::LineSegment> lines =
    math::geometry::getLineSegments(points, true);
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
  EXPECT_EQ(lines.size(), size_t(0));
}

TEST(LineSegment, getLineSegmentsVectorIdentical)
{
  geometry_msgs::msg::Point point = makePoint(1.0, 2.0, 3.0);
  const std::vector<geometry_msgs::msg::Point> points{point, point, point, point};
  const std::vector<math::geometry::LineSegment> lines =
    math::geometry::getLineSegments(points, false);
  EXPECT_EQ(lines.size(), size_t(3));
  EXPECT_POINT_EQ(lines[0].start_point, point);
  EXPECT_POINT_EQ(lines[0].end_point, point);
  EXPECT_POINT_EQ(lines[1].start_point, point);
  EXPECT_POINT_EQ(lines[1].end_point, point);
  EXPECT_POINT_EQ(lines[2].start_point, point);
  EXPECT_POINT_EQ(lines[2].end_point, point);
}

TEST(LineSegment, getLineSegmentsVectorIdentical_closeStartEnd)
{
  geometry_msgs::msg::Point point = makePoint(1.0, 2.0, 3.0);
  const std::vector<geometry_msgs::msg::Point> points{point, point, point, point};
  const std::vector<math::geometry::LineSegment> lines =
    math::geometry::getLineSegments(points, true);
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

TEST(LineSegment, getSValue)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 3.0));
  const auto s = line.getSValue(makePose(2.0, 2.0, 2.0), 1.0, false);
  EXPECT_TRUE(s);
  if (s) {
    EXPECT_DOUBLE_EQ(s.value(), 2.0 / 3.0);
  }
}

TEST(LineSegment, getSValue_denormalize)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 3.0));
  const auto s = line.getSValue(makePose(2.0, 2.0, 2.0), 1.0, true);
  EXPECT_TRUE(s);
  if (s) {
    EXPECT_DOUBLE_EQ(s.value(), std::hypot(2.0, 2.0, 2.0));
  }
}

TEST(LineSegment, getSValue_outOfRange)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 3.0));
  const auto s = line.getSValue(makePose(4.0, 4.0, 4.0), 1.0, false);
  EXPECT_FALSE(s);
}

TEST(LineSegment, getSValue_outOfRangeDenormalize)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 3.0));
  const auto s = line.getSValue(makePose(4.0, 4.0, 4.0), 1.0, true);
  EXPECT_FALSE(s);
}

TEST(LineSegment, getSValue_parallel)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 0.0));
  const auto s = line.getSValue(
    makePose(
      1.0, 0.0, 0.0,
      math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, 0.0, M_PI_4 * 3.0))),
    1000.0, false);
  EXPECT_FALSE(s);
}

TEST(LineSegment, getSValue_parallelDenormalize)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 0.0));
  const auto s = line.getSValue(
    makePose(
      1.0, 0.0, 0.0,
      math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, 0.0, M_PI_4 * 3.0))),
    1000.0, true);
  EXPECT_FALSE(s);
}

TEST(LineSegment, GetPoint_outOfBounds_denormalized)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 1.0), makePoint(3.0, 2.0, -1.0));
  ASSERT_DOUBLE_EQ(line.getLength(), 6.0);

  EXPECT_THROW(line.getPoint(7.0, true), common::SimulationError);
  EXPECT_THROW(line.getPoint(-1.0, true), common::SimulationError);
}

TEST(LineSegment, getPoint_outOfBounds_normalized)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 1.0), makePoint(3.0, 2.0, -1.0));
  ASSERT_DOUBLE_EQ(line.getLength(), 6.0);

  EXPECT_THROW(line.getPoint(1.1, false), common::SimulationError);
  EXPECT_THROW(line.getPoint(-0.1, false), common::SimulationError);
}

TEST(LineSegment, getPoint_inside_denormalized)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 1.0), makePoint(3.0, 2.0, -1.0));
  ASSERT_DOUBLE_EQ(line.getLength(), 6.0);

  EXPECT_POINT_EQ(line.getPoint(0.0, true), makePoint(-1.0, -2.0, 1.0));
  EXPECT_POINT_EQ(line.getPoint(3.0, true), makePoint(1.0, 0.0, 0.0));
  EXPECT_POINT_EQ(line.getPoint(6.0, true), makePoint(3.0, 2.0, -1.0));
}

TEST(LineSegment, getPoint_inside_normalized)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 1.0), makePoint(3.0, 2.0, -1.0));
  ASSERT_DOUBLE_EQ(line.getLength(), 6.0);

  EXPECT_POINT_EQ(line.getPoint(0.0, false), makePoint(-1.0, -2.0, 1.0));
  EXPECT_POINT_EQ(line.getPoint(0.5, false), makePoint(1.0, 0.0, 0.0));
  EXPECT_POINT_EQ(line.getPoint(1.0, false), makePoint(3.0, 2.0, -1.0));
}

TEST(LineSegment, getPose_denormalized)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 0.0), makePoint(3.0, 2.0, 4.0));
  const double length = 4.0 * std::sqrt(3.0);
  ASSERT_DOUBLE_EQ(line.getLength(), length);

  EXPECT_POSE_EQ(
    line.getPose(0.0 * length, true, false),
    makePose(
      -1.0, -2.0, 0.0,
      math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, 0.0, M_PI_4))));

  EXPECT_POSE_EQ(
    line.getPose(0.5 * length, true, false),
    makePose(
      1.0, 0.0, 2.0, math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, 0.0, M_PI_4))));

  EXPECT_POSE_EQ(
    line.getPose(1.0 * length, true, false),
    makePose(
      3.0, 2.0, 4.0, math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, 0.0, M_PI_4))));
}

TEST(LineSegment, getPose_normalized)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 0.0), makePoint(3.0, 2.0, 4.0));
  const double length = 4.0 * std::sqrt(3.0);
  ASSERT_DOUBLE_EQ(line.getLength(), length);

  EXPECT_POSE_EQ(
    line.getPose(0.0, false, false),
    makePose(
      -1.0, -2.0, 0.0,
      math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, 0.0, M_PI_4))));

  EXPECT_POSE_EQ(
    line.getPose(0.5, false, false),
    makePose(
      1.0, 0.0, 2.0, math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, 0.0, M_PI_4))));

  EXPECT_POSE_EQ(
    line.getPose(1.0, false, false),
    makePose(
      3.0, 2.0, 4.0, math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, 0.0, M_PI_4))));
}

TEST(LineSegment, getPose_pitch)
{
  const auto line = math::geometry::LineSegment(
    makePoint(-1.0, -2.0, 0.0 * std::sqrt(2.0)), makePoint(3.0, 2.0, 4.0 * std::sqrt(2.0)));
  const double length = 8.0;
  ASSERT_DOUBLE_EQ(line.getLength(), length);

  EXPECT_POSE_EQ(
    line.getPose(0.0 * length, true, true),
    makePose(
      -1.0, -2.0, 0.0 * std::sqrt(2.0),
      math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, -M_PI_4, M_PI_4))));

  EXPECT_POSE_EQ(
    line.getPose(0.5 * length, true, true),
    makePose(
      1.0, 0.0, 2.0 * std::sqrt(2.0),
      math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, -M_PI_4, M_PI_4))));

  EXPECT_POSE_EQ(
    line.getPose(1.0 * length, true, true),
    makePose(
      3.0, 2.0, 4.0 * std::sqrt(2.0),
      math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, -M_PI_4, M_PI_4))));
}

TEST(LineSegment, isIntersect2DDisjoint)
{
  const math::geometry::LineSegment line0(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  const math::geometry::LineSegment line1(makePoint(1.0, 0.0), makePoint(2.0, 1.0));
  EXPECT_FALSE(line0.isIntersect2D(line1));
}

TEST(LineSegment, isIntersect2DIntersect)
{
  const math::geometry::LineSegment line0(makePoint(1.0, 1.0), makePoint(2.0, 1.0));
  const math::geometry::LineSegment line1(makePoint(1.0, 0.0), makeVector(1.0, 1.0), 3.0);
  EXPECT_TRUE(line0.isIntersect2D(line1));
}

TEST(LineSegment, isIntersect2DIdentical)
{
  const math::geometry::LineSegment line(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  EXPECT_TRUE(line.isIntersect2D(line));
}

TEST(LineSegment, isIntersect2D_pointInside)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 0.0), makePoint(3.0, 2.0, 4.0));

  EXPECT_TRUE(line.isIntersect2D(makePoint(0.0, -1.0, 1.0)));
  EXPECT_TRUE(line.isIntersect2D(makePoint(0.0, -1.0, 0.0)));
}

TEST(LineSegment, isIntersect2D_pointOutside)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 0.0), makePoint(3.0, 2.0, 4.0));

  EXPECT_FALSE(line.isIntersect2D(makePoint(-1.0, -1.0, 1.0)));
  EXPECT_FALSE(line.isIntersect2D(makePoint(0.0, 89.0, 97.0)));

  EXPECT_FALSE(line.isIntersect2D(makePoint(-3.0, 0.0, 0.0)));
  EXPECT_FALSE(line.isIntersect2D(makePoint(4.0, 0.0, 0.0)));
}

TEST(LineSegment, isIntersect2D_pointCollinear)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 0.0), makePoint(3.0, 2.0, 4.0));

  EXPECT_FALSE(line.isIntersect2D(makePoint(-2.0, -3.0, -1.0)));
  EXPECT_FALSE(line.isIntersect2D(makePoint(-2.0, -3.0, 0.0)));

  EXPECT_FALSE(line.isIntersect2D(makePoint(4.0, 3.0, 5.0)));
  EXPECT_FALSE(line.isIntersect2D(makePoint(4.0, 3.0, 0.0)));
}

TEST(LineSegment, isIntersect2D_pointOnEnd)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 0.0), makePoint(3.0, 2.0, 4.0));

  EXPECT_TRUE(line.isIntersect2D(makePoint(-1.0, -2.0, 0.0)));
  EXPECT_TRUE(line.isIntersect2D(makePoint(-1.0, -2.0, 7.0)));

  EXPECT_TRUE(line.isIntersect2D(makePoint(3.0, 2.0, 4.0)));
  EXPECT_TRUE(line.isIntersect2D(makePoint(3.0, 2.0, -5.0)));
}

TEST(LineSegment, getIntersection2DSValue_line_vertical)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 0.0), makePoint(-1.0, 2.0, 0.0));
  {
    const auto s_value = line.getIntersection2DSValue(
      math::geometry::LineSegment(makePoint(-2.0, -2.0, 0.0), makePoint(1.0, 1.0, 0.0)), true);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 1.0);
  }
  {
    const auto s_value = line.getIntersection2DSValue(
      math::geometry::LineSegment(makePoint(-2.0, -2.0, 0.0), makePoint(1.0, 1.0, 0.0)), false);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 0.25);
  }
}

TEST(LineSegment, getIntersection2DSValue_line_horizontal)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, 2.0, 0.0), makePoint(1.0, 2.0, 0.0));
  {
    const auto s_value = line.getIntersection2DSValue(
      math::geometry::LineSegment(makePoint(-1.0, 0.0, 0.0), makePoint(1.0, 4.0, 0.0)), true);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 1.0);
  }
  {
    const auto s_value = line.getIntersection2DSValue(
      math::geometry::LineSegment(makePoint(-1.0, 0.0, 0.0), makePoint(1.0, 4.0, 0.0)), false);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 0.5);
  }
}

TEST(LineSegment, getIntersection2DSValue_line_bounds)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, 1.0, 0.0), makePoint(1.0, 3.0, 0.0));
  {
    const auto s_value = line.getIntersection2DSValue(
      math::geometry::LineSegment(makePoint(-2.0, 2.0, 0.0), makePoint(0.0, 0.0, 0.0)), false);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 0.0);
  }
  {
    const auto s_value = line.getIntersection2DSValue(
      math::geometry::LineSegment(makePoint(1.0, 3.0, 0.0), makePoint(2.0, 2.0, 0.0)), false);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 1.0);
  }
}

TEST(LineSegment, getIntersection2DSValue_line_outside)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, 1.0, 0.0), makePoint(1.0, 3.0, 0.0));

  EXPECT_FALSE(
    line
      .getIntersection2DSValue(
        math::geometry::LineSegment(makePoint(-2.0, 1.0, 0.0), makePoint(0.0, 0.0, 0.0)), false)
      .has_value());

  EXPECT_FALSE(
    line
      .getIntersection2DSValue(
        math::geometry::LineSegment(makePoint(1.0, 4.0, 0.0), makePoint(2.0, 2.0, 0.0)), false)
      .has_value());
}

TEST(LineSegment, getIntersection2DSValue_line_collinear)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, 1.0, 0.0), makePoint(1.0, 3.0, 0.0));

  EXPECT_THROW(line.getIntersection2DSValue(line, false), common::SimulationError);

  EXPECT_FALSE(
    line
      .getIntersection2DSValue(
        math::geometry::LineSegment(makePoint(3.0, 5.0, 0.0), makePoint(5.0, 7.0, 0.0)), false)
      .has_value());
}

TEST(LineSegment, getIntersection2DSValue_point_vertical)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, -2.0, 0.0), makePoint(-1.0, 2.0, 0.0));

  {
    const auto s_value = line.getIntersection2DSValue(makePoint(-1.0, 0.0, 0.0), true);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 2.0);
  }
  {
    const auto s_value = line.getIntersection2DSValue(makePoint(-1.0, 0.0, 0.0), false);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 0.5);
  }
  {
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-101.0, 0.0, 0.0), true).has_value());
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(103.0, 0.0, 0.0), false).has_value());
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-1.0, -107.0, 0.0), true).has_value());
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-1.0, 109.0, 0.0), false).has_value());
  }
}

TEST(LineSegment, getIntersection2DSValue_point_horizontal)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-1.0, 2.0, 0.0), makePoint(1.0, 2.0, 0.0));

  {
    const auto s_value = line.getIntersection2DSValue(makePoint(0.0, 2.0, 0.0), true);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 1.0);
  }
  {
    const auto s_value = line.getIntersection2DSValue(makePoint(0.0, 2.0, 0.0), false);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 0.5);
  }
  {
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-113.0, 2.0, 0.0), true).has_value());
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(127.0, 2.0, 0.0), false).has_value());
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(0.0, -131.0, 0.0), true).has_value());
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(0.0, 137.0, 0.0), false).has_value());
  }
}

TEST(LineSegment, getIntersection2DSValue_point_bounds)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-2.0, -2.0, 0.0), makePoint(1.0, 4.0, 0.0));

  {
    const auto s_value = line.getIntersection2DSValue(makePoint(-2.0, -2.0, 0.0), true);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 0.0);
  }
  {
    const auto s_value = line.getIntersection2DSValue(makePoint(-2.0, -2.0, 0.0), false);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 0.0);
  }
  {
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-139.0, -2.0, 0.0), true).has_value());
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(149.0, -2.0, 0.0), false).has_value());
  }

  {
    const auto s_value = line.getIntersection2DSValue(makePoint(1.0, 4.0, 0.0), true);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 3.0 * std::sqrt(5));
  }
  {
    const auto s_value = line.getIntersection2DSValue(makePoint(1.0, 4.0, 0.0), false);
    ASSERT_TRUE(s_value.has_value());
    EXPECT_DOUBLE_EQ(s_value.value(), 1.0);
  }
  {
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-151.0, 4.0, 0.0), true).has_value());
    EXPECT_FALSE(line.getIntersection2DSValue(makePoint(157.0, 4.0, 0.0), false).has_value());
  }
}

TEST(LineSegment, getIntersection2DSValue_point_outside)
{
  const auto line =
    math::geometry::LineSegment(makePoint(-2.0, -2.0, 0.0), makePoint(1.0, 4.0, 0.0));

  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-3.0, 1.0, 0.0), true).has_value());
  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-1.0, 1.0, 0.0), true).has_value());
  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(0.0, 1.0, 0.0), true).has_value());
  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(2.0, 1.0, 0.0), true).has_value());

  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-0.5, -5.0, 0.0), true).has_value());
  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-0.5, -1.0, 0.0), true).has_value());
  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-0.5, 3.0, 0.0), true).has_value());
  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-0.5, 7.0, 0.0), true).has_value());

  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(7.0, 7.0, 0.0), true).has_value());
  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(7.0, -7.0, 0.0), true).has_value());
  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-7.0, 7.0, 0.0), true).has_value());
  EXPECT_FALSE(line.getIntersection2DSValue(makePoint(-7.0, -7.0, 0.0), true).has_value());
}

TEST(LineSegment, getIntersection2DDisjoint)
{
  const math::geometry::LineSegment line0(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  const math::geometry::LineSegment line1(makePoint(1.0, 0.0), makePoint(2.0, 1.0));
  EXPECT_FALSE(line0.getIntersection2D(line1));
}

TEST(LineSegment, getIntersection2DIntersect)
{
  const auto line0 = math::geometry::LineSegment(makePoint(0.0, 0.0), makePoint(1.0, 1.0));
  const auto line1 = math::geometry::LineSegment(makePoint(1.0, 0.0), makePoint(0.0, 1.0));

  const auto p0 = line0.getIntersection2D(line1);
  const auto p1 = line1.getIntersection2D(line0);

  ASSERT_TRUE(p0.has_value());
  ASSERT_TRUE(p1.has_value());
  EXPECT_POINT_EQ(p0.value(), makePoint(0.5, 0.5));
  EXPECT_POINT_EQ(p1.value(), makePoint(0.5, 0.5));
}

TEST(LineSegment, getIntersection2DIdentical)
{
  const auto line = math::geometry::LineSegment(makePoint(0.0, 0.0), makePoint(1.0, 1.0));

  EXPECT_THROW(line.getIntersection2D(line), common::SimulationError);
}

TEST(LineSegment, getSValue)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 3.0));
  const auto s = line.getSValue(makePose(2.0, 2.0, 2.0), 1.0, false);
  EXPECT_TRUE(s);
  if (s) {
    EXPECT_DOUBLE_EQ(s.value(), 2.0 / 3.0);
  }
}

TEST(LineSegment, getSValue_denormalize)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 3.0));
  const auto s = line.getSValue(makePose(2.0, 2.0, 2.0), 1.0, true);
  EXPECT_TRUE(s);
  if (s) {
    EXPECT_DOUBLE_EQ(s.value(), std::hypot(2.0, 2.0, 2.0));
  }
}

TEST(LineSegment, getSValue_outOfRange)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 3.0));
  const auto s = line.getSValue(makePose(4.0, 4.0, 4.0), 1.0, false);
  EXPECT_FALSE(s);
}

TEST(LineSegment, getSValue_outOfRangeDenormalize)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 3.0));
  const auto s = line.getSValue(makePose(4.0, 4.0, 4.0), 1.0, true);
  EXPECT_FALSE(s);
}

TEST(LineSegment, getSValue_parallel)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 0.0));
  const auto s = line.getSValue(
    makePose(
      1.0, 0.0, 0.0,
      math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, 0.0, M_PI_4 * 3.0))),
    1000.0, false);
  EXPECT_FALSE(s);
}

TEST(LineSegment, getSValue_parallelDenormalize)
{
  math::geometry::LineSegment line(makePoint(0.0, 0.0, 0.0), makePoint(3.0, 3.0, 0.0));
  const auto s = line.getSValue(
    makePose(
      1.0, 0.0, 0.0,
      math::geometry::convertEulerAngleToQuaternion(makeVector(0.0, 0.0, M_PI_4 * 3.0))),
    1000.0, true);
  EXPECT_FALSE(s);
}

TEST(LineSegment, getVector)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0));
  EXPECT_VECTOR3_EQ(line.getVector(), makeVector(1.0, 1.0, 1.0));
}

TEST(LineSegment, getVectorZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_VECTOR3_EQ(line.getVector(), makeVector(0.0, 0.0, 0.0));
}

TEST(LineSegment, getNormalVector)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0));
  EXPECT_VECTOR3_EQ(line.getNormalVector(), makeVector(-1.0, 1.0, 0.0));
}

TEST(LineSegment, getNormalVector_zeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_VECTOR3_EQ(line.getNormalVector(), makeVector(0.0, 0.0, 0.0));
}

TEST(LineSegment, get2DVector)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0));
  EXPECT_VECTOR3_EQ(line.get2DVector(), makeVector(1.0, 1.0, 0.0));
}

TEST(LineSegment, get2DVectorZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_VECTOR3_EQ(line.get2DVector(), makeVector(0.0, 0.0, 0.0));
}

TEST(LineSegment, getLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0));
  EXPECT_DOUBLE_EQ(line.getLength(), std::sqrt(3.0));
}

TEST(LineSegment, getLengthZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_DOUBLE_EQ(line.getLength(), 0.0);
}

TEST(LineSegment, get2DLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0));
  EXPECT_DOUBLE_EQ(line.get2DLength(), std::sqrt(2.0));
}

TEST(LineSegment, get2DLengthZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_DOUBLE_EQ(line.get2DLength(), 0.0);
}

TEST(LineSegment, getSlope)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(3.0, 3.0, 4.0));
  EXPECT_DOUBLE_EQ(line.getSlope(), 0.5);
}

TEST(LineSegment, getSlopeZeroLength)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(1.0, 2.0, 3.0));
  EXPECT_TRUE(std::isnan(line.getSlope()));
}

TEST(LineSegment, getSquaredDistanceIn2D)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(3.0, 4.0, 5.0));
  EXPECT_DOUBLE_EQ(line.getSquaredDistanceIn2D(makePoint(0.0, 1.0, 2.0), 0.5, false), 8.0);
}

TEST(LineSegment, getSquaredDistanceIn2D_denormalize)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(3.0, 4.0, 5.0));
  EXPECT_DOUBLE_EQ(
    line.getSquaredDistanceIn2D(makePoint(0.0, 1.0, 2.0), std::sqrt(3.0), true), 8.0);
}

TEST(LineSegment, getSquaredDistanceVector)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(3.0, 4.0, 5.0));
  EXPECT_VECTOR3_EQ(
    line.getSquaredDistanceVector(makePoint(0.0, 1.0, 2.0), 0.5, false),
    makeVector(-2.0, -2.0, -2.0));
}

TEST(LineSegment, getSquaredDistanceVector_denormalize)
{
  const math::geometry::LineSegment line(makePoint(1.0, 2.0, 3.0), makePoint(3.0, 4.0, 5.0));
  EXPECT_VECTOR3_EQ(
    line.getSquaredDistanceVector(makePoint(0.0, 1.0, 2.0), std::sqrt(3.0), true),
    makeVector(-2.0, -2.0, -2.0));
}

TEST(LineSegment, getLineSegments)
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0), makePoint(3.0, 4.0, 5.0),
    makePoint(4.0, 5.0, 6.0)};
  const std::vector<math::geometry::LineSegment> lines =
    math::geometry::getLineSegments(points, false);
  EXPECT_EQ(lines.size(), size_t(3));
  EXPECT_POINT_EQ(lines[0].start_point, points[0]);
  EXPECT_POINT_EQ(lines[0].end_point, points[1]);
  EXPECT_POINT_EQ(lines[1].start_point, points[1]);
  EXPECT_POINT_EQ(lines[1].end_point, points[2]);
  EXPECT_POINT_EQ(lines[2].start_point, points[2]);
  EXPECT_POINT_EQ(lines[2].end_point, points[3]);
}

TEST(LineSegment, getLineSegments_closeStartEnd)
{
  const std::vector<geometry_msgs::msg::Point> points{
    makePoint(1.0, 2.0, 3.0), makePoint(2.0, 3.0, 4.0), makePoint(3.0, 4.0, 5.0),
    makePoint(4.0, 5.0, 6.0)};
  const std::vector<math::geometry::LineSegment> lines =
    math::geometry::getLineSegments(points, true);
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
  EXPECT_EQ(lines.size(), size_t(0));
}

TEST(LineSegment, getLineSegmentsVectorIdentical)
{
  geometry_msgs::msg::Point point = makePoint(1.0, 2.0, 3.0);
  const std::vector<geometry_msgs::msg::Point> points{point, point, point, point};
  const std::vector<math::geometry::LineSegment> lines =
    math::geometry::getLineSegments(points, false);
  EXPECT_EQ(lines.size(), size_t(3));
  EXPECT_POINT_EQ(lines[0].start_point, point);
  EXPECT_POINT_EQ(lines[0].end_point, point);
  EXPECT_POINT_EQ(lines[1].start_point, point);
  EXPECT_POINT_EQ(lines[1].end_point, point);
  EXPECT_POINT_EQ(lines[2].start_point, point);
  EXPECT_POINT_EQ(lines[2].end_point, point);
}

TEST(LineSegment, getLineSegmentsVectorIdentical_closeStartEnd)
{
  geometry_msgs::msg::Point point = makePoint(1.0, 2.0, 3.0);
  const std::vector<geometry_msgs::msg::Point> points{point, point, point, point};
  const std::vector<math::geometry::LineSegment> lines =
    math::geometry::getLineSegments(points, true);
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
