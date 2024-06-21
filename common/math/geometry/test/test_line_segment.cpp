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
#include <scenario_simulator_exception/exception.hpp>

#include "expect_eq_macros.hpp"
#include "test_utils.hpp"

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

/// @brief In this test case, testing the `LineSegment::getPoint` function can find the point on the line segment with start point (x,y,z) = (0,0,0) and end point (x,y,z) = (1,1,1) in the cartesian coordinate system. (variable name `line`).
TEST(LineSegment, GetPoint)
{
  {
    math::geometry::LineSegment line(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(1));
    /// @note If s = 0 and denormalize_s = false, the return value of the `getPoint` function should be a start point of the `line`.
    /// If the `denormalize_s = false`, the return value `s` of the function is normalized and in range `s = [0,1]`.
    // [Snippet_getPoint_with_s_0]
    EXPECT_POINT_EQ(
      line.getPoint(0, false), geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0));
    // [Snippet_getPoint_with_s_0]
    /// @snippet test/test_line_segment.cpp Snippet_getPoint_with_s_0

    /// @note If s = 0 and denormalize_s = false, the return value of the `LineSegment::getPoint` function should be a end point of the `line`.
    /// If the `denormalize_s = false`, the return value `s` of the function is normalized and in range `s = [0,1]`.
    // [Snippet_getPoint_with_s_1]
    EXPECT_POINT_EQ(
      line.getPoint(1, false), geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(1));
    // [Snippet_getPoint_with_s_1]
    /// @snippet test/test_line_segment.cpp Snippet_getPoint_with_s_1

    /// @note If s = sqrt(3) and denormalize_s = true, the return value of the `LineSegment::getPoint` function should be a end point of the `line`.
    /// If the `denormalize_s = true`, the return value `s` is denormalized and it returns the value `s` times the length of the curve.
    // [Snippet_getPoint_with_sqrt_3_denormalized]
    EXPECT_POINT_EQ(
      line.getPoint(std::sqrt(3), true),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(1).z(1));
    // [Snippet_getPoint_with_sqrt_3_denormalized]
    /// @snippet test/test_line_segment.cpp Snippet_getPoint_with_sqrt_3_denormalized

    /// @note If s is not in range s = [0,1], testing the `LineSegment::getPoint` function can throw error. If s is not in range s = [0,1], it means the point is not on the line.
    // [Snippet_getPoint_out_of_range]
    EXPECT_THROW(line.getPoint(2, false), common::SimulationError);
    EXPECT_THROW(line.getPoint(-1, false), common::SimulationError);
    // [Snippet_getPoint_out_of_range]
    /// @snippet test/test_line_segment.cpp Snippet_getPoint_out_of_range

    /// @note If s is not in range s = [0,sqrt(3)] and denormalize, testing the `LineSegment::getPoint` function can throw error. If s is not in range s = [0,sqrt(3)], it means the point is not on the line.
    // [Snippet_getPoint_out_of_range_with_denormalize]
    EXPECT_THROW(line.getPoint(2, true), common::SimulationError);
    EXPECT_THROW(line.getPoint(-1, true), common::SimulationError);
    EXPECT_NO_THROW(line.getPoint(0, true));
    EXPECT_NO_THROW(line.getPoint(std::sqrt(3.0), true));
    // [Snippet_getPoint_out_of_range_with_denormalize]
    /// @snippet test/test_line_segment.cpp Snippet_getPoint_out_of_range_with_denormalize
  }
}

/// @brief In this test case, testing the `LineSegment::getPose` function can find the pose on the line segment with start point (x,y,z) = (0,0,0) and end point (x,y,z) = (0,0,1) in the cartesian coordinate system. (variable name `line`).
TEST(LineSegment, GetPose)
{
  {
    math::geometry::LineSegment line(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(1));
    /// @note When the `s = 0`(1st argument of the `LineSegment::getPose` function), the return value of the `LineSegment::getPose` function should be a start point of the `line`.
    /// Orientation should be defined by the direction of the `line`, so the orientation should be parallel to the z axis.
    // [Snippet_getPose_with_s_0]
    EXPECT_POSE_EQ(
      line.getPose(0, false, false),
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0))
        .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1)));
    EXPECT_POSE_EQ(
      line.getPose(0, false, true),
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0))
        .orientation(math::geometry::convertEulerAngleToQuaternion(
          geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0).y(-M_PI * 0.5).z(0))));
    // [Snippet_getPose_with_s_0]
    /// @snippet test/test_line_segment.cpp Snippet_getPose_with_s_0

    /// @note When the `s = 1`(1st argument of the `LineSegment::getPose` function), the return value of the `LineSegment::getPose` function should be an end point of the `line`.
    /// Orientation should be defined by the direction of the `line`, so the orientation should be parallel to the z axis.
    // [Snippet_getPose_with_s_1]
    EXPECT_POSE_EQ(
      line.getPose(1, false, false),
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(1))
        .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1)));
    EXPECT_POSE_EQ(
      line.getPose(1, false, true),
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(1))
        .orientation(math::geometry::convertEulerAngleToQuaternion(
          geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0).y(-M_PI * 0.5).z(0))));
    // [Snippet_getPose_with_s_1]
    /// @snippet test/test_line_segment.cpp Snippet_getPose_with_s_1
  }
}

/// @brief In this test case, we check collision with the point and the line segment with start point (x,y,z) = (0,-1,0) and end point (x,y,z) = (0,1,0) in the cartesian coordinate system. (variable name `line`).
TEST(LineSegment, isIntersect2D)
{
  {
    math::geometry::LineSegment line(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-1).z(0),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0));
    /// @note Testing the `LineSegment::isIntersect2D` function can find intersection with point (0,0,0) and `line`.
    // [Snippet_isIntersect2D_with_point_0_0_0]
    EXPECT_TRUE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0)));
    // [Snippet_isIntersect2D_with_point_0_0_0]
    /// @snippet test/test_line_segment.cpp Snippet_isIntersect2D_with_point_0_0_0

    /// @note Testing the `LineSegment::isIntersect2D` function can find intersection with point (0,0,-1)
    // [Snippet_isIntersect2D_with_point_0_0_-1]
    EXPECT_TRUE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(-1)));
    // [Snippet_isIntersect2D_with_point_0_0_-1]
    /// @snippet test/test_line_segment.cpp Snippet_isIntersect2D_with_point_0_0_-1

    /// @note Testing the `LineSegment::isIntersect2D` function can find intersection with point (0,1,0) and `line`.
    // [Snippet_isIntersect2D_with_point_0_1_0]
    EXPECT_TRUE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0)));
    // [Snippet_isIntersect2D_with_point_0_1_0]
    /// @snippet test/test_line_segment.cpp Snippet_isIntersect2D_with_point_0_1_0

    /// @note Testing the `LineSegment::isIntersect2D` function can find that there is no intersection with point (0,2,0) and `line`.
    // [Snippet_isIntersect2D_with_point_0_2_0]
    EXPECT_FALSE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(2).z(0)));
    // [Snippet_isIntersect2D_with_point_0_2_0]
    /// @snippet test/test_line_segment.cpp Snippet_isIntersect2D_with_point_0_2_0

    /// @note Testing the `LineSegment::isIntersect2D` function can find that there is no intersection with point (0,-2,0) and `line`.
    // [Snippet_isIntersect2D_with_point_0_-2_0]
    EXPECT_FALSE(
      line.isIntersect2D(geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-2).z(0)));
    // [Snippet_isIntersect2D_with_point_0_-2_0]
    /// @snippet test/test_line_segment.cpp Snippet_isIntersect2D_with_point_0_-2_0
  }
}

/// @brief In this test case, we check collision with the line segment with start point (x,y,z) = (0,-1,0) and end point (x,y,z) = (0,1,0) in the cartesian coordinate system. (variable name `line`).
TEST(LineSegment, getIntersection2DSValue)
{
  {
    math::geometry::LineSegment line(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-1).z(0),
      geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0));
    /**
     * @note Testing the `LineSegment::getIntersection2DSValue` function can find a collision with the point with (x,y,z) = (0,0,0) in the cartesian coordinate system.
     * In the frenet coordinate system along the `line`, the s value should be 0.5.
     * If so, the variable `collision_s` should be `std::optional<double>(0.5)`.
     */
    // [Snippet_getIntersection2DSValue_with_point_0_0_0]
    {
      const auto collision_s = line.getIntersection2DSValue(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0), false);
      EXPECT_TRUE(collision_s);
      if (collision_s) {
        EXPECT_DOUBLE_EQ(collision_s.value(), 0.5);
      }
    }
    // [Snippet_getIntersection2DSValue_with_point_0_0_0]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2DSValue_with_point_0_0_0

    /**
     * @note Testing the `LineSegment::getIntersection2DSValue` function can find a collision with the point with (x,y,z) = (0,1,0) in the cartesian coordinate system.
     * In the frenet coordinate system along the `line`, the s value should be 1.0.
     * If so, the variable `collision_s` should be `std::optional<double>(1.0)`.
     */
    // [Snippet_getIntersection2DSValue_with_point_0_1_0]
    {
      const auto collision_s = line.getIntersection2DSValue(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0), false);
      EXPECT_TRUE(collision_s);
      if (collision_s) {
        EXPECT_DOUBLE_EQ(collision_s.value(), 1.0);
      }
    }
    // [Snippet_getIntersection2DSValue_with_point_0_1_0]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2DSValue_with_point_0_1_0

    /**
     * @note Testing the `LineSegment::getIntersection2DSValue` function can find a collision with the point with (x,y,z) = (0,1,0) in the cartesian coordinate system.
     * In the frenet coordinate system along the `line`, the s value should be 1.0.
     * And, the 2nd argument of the `LineSegment::getIntersection2DSValue` (denormalized_s) is true, so the return value should be 1.0 (normalized s value.) * 2.0 (length of the `line`) = 2.0.
     * If so, the variable `collision_s` should be `std::optional<double>(2.0)`.
     */
    // [Snippet_getIntersection2DSValue_with_point_0_1_0_denormalized]
    {
      const auto collision_s = line.getIntersection2DSValue(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(1).z(0), true);
      EXPECT_TRUE(collision_s);
      if (collision_s) {
        EXPECT_DOUBLE_EQ(collision_s.value(), 2.0);
      }
    }
    // [Snippet_getIntersection2DSValue_with_point_0_1_0_denormalized]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2DSValue_with_point_0_1_0_denormalized

    /**
     * @note Testing the `LineSegment::getIntersection2DSValue` function can find a collision with the point with (x,y,z) = (0,0,0) in the cartesian coordinate system.
     * In the frenet coordinate system along the `line`, the s value should be 0.5.
     * And, the 2nd argument of the `LineSegment::getIntersection2DSValue` (denormalized_s) is true, so the return value should be 0.5 (normalized s value.) * 2.0 (length of the `line`) = 1.0.
     * If so, the variable `collision_s` should be `std::optional<double>(1.0)`.
     */
    // [Snippet_getIntersection2DSValue_with_point_0_0_0_denormalized]
    {
      const auto collision_s = line.getIntersection2DSValue(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0), true);
      EXPECT_TRUE(collision_s);
      if (collision_s) {
        EXPECT_DOUBLE_EQ(collision_s.value(), 1.0);
      }
    }
    // [Snippet_getIntersection2DSValue_with_point_0_0_0_denormalized]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2DSValue_with_point_0_0_0_denormalized

    /**
     * @note Testing the `LineSegment::getIntersection2DSValue` function can find that the point with (x,y,z) = (1,0,0) in the cartesian coordinate system does not collide to `line.`.
     * If the function works valid, the variable `collision_s` should be `std::nullopt`.
     */
    // [Snippet_getIntersection2DSValue_with_point_1_0_0]
    {
      const auto collision_s = line.getIntersection2DSValue(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(0).z(0), false);
      EXPECT_FALSE(collision_s);
    }
    // [Snippet_getIntersection2DSValue_with_point_1_0_0]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2DSValue_with_point_1_0_0

    {  // parallel no denormalize
      EXPECT_THROW(
        line.getIntersection2DSValue(
          math::geometry::LineSegment(makePoint(0.0, -1.0), makePoint(0.0, 1.0)), false),
        common::SimulationError);
    }
    {  // parallel denormalize
      EXPECT_THROW(
        line.getIntersection2DSValue(
          math::geometry::LineSegment(makePoint(0.0, -1.0), makePoint(0.0, 1.0)), true),
        common::SimulationError);
    }
    {  // intersect no denormalize
      const auto collision_s = line.getIntersection2DSValue(
        math::geometry::LineSegment(makePoint(-1.0, 0.5), makePoint(1.0, 0.5)), false);
      EXPECT_TRUE(collision_s);
      if (collision_s) {
        EXPECT_DOUBLE_EQ(collision_s.value(), 0.75);
      }
    }
    {  // intersect denormalize
      const auto collision_s = line.getIntersection2DSValue(
        math::geometry::LineSegment(makePoint(-1.0, 0.5), makePoint(1.0, 0.5)), true);
      EXPECT_TRUE(collision_s);
      if (collision_s) {
        EXPECT_DOUBLE_EQ(collision_s.value(), 1.5);
      }
    }
    {  // no intersect no denormalize
      const auto collision_s = line.getIntersection2DSValue(
        math::geometry::LineSegment(makePoint(-1.0, 1.5), makePoint(1.0, 1.5)), false);
      EXPECT_FALSE(collision_s);
    }
    {  // no intersect denormalize
      const auto collision_s = line.getIntersection2DSValue(
        math::geometry::LineSegment(makePoint(-1.0, 1.5), makePoint(1.0, 1.5)), true);
      EXPECT_FALSE(collision_s);
    }

    /**
     * @note Testing the `LineSegment::getIntersection2D` function can throw error getting intersection with exact same line segment.
     * In this case, any s value can be a intersection point, so we cannot return single value.
     */
    // [Snippet_getIntersection2D_with_line]
    {
      EXPECT_THROW(line.getIntersection2D(line), common::SimulationError);
    }
    // [Snippet_getIntersection2D_with_line]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2D_with_line

    /**
     * @note Testing the `LineSegment::getIntersection2D` function can throw error getting intersection with part of the line segment `line`.
     * In this case, any s value can be a intersection point, so we cannot return single value.
     */
    // [Snippet_getIntersection2D_with_part_of_line]
    {
      EXPECT_THROW(
        line.getIntersection2D(math::geometry::LineSegment(
          geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(-1).z(0),
          geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0))),
        common::SimulationError);
    }
    // [Snippet_getIntersection2D_with_part_of_line]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2D_with_part_of_line

    /// @note @note Testing the `LineSegment::getIntersection2D` function can find collision with the line segment with start point (x,y,z) = (1,0,0) and end point (x,y,z) = (-1,0,0) in the cartesian coordinate system and `line`.
    // [Snippet_getIntersection2D_line_1_0_0_-1_0_0]
    {
      const auto point = line.getIntersection2D(math::geometry::LineSegment(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(1).y(0).z(0),
        geometry_msgs::build<geometry_msgs::msg::Point>().x(-1).y(0).z(0)));
      EXPECT_TRUE(point);
      if (point) {
        EXPECT_POINT_EQ(
          point.value(), geometry_msgs::build<geometry_msgs::msg::Point>().x(0).y(0).z(0));
      }
    }
    // [Snippet_getIntersection2D_line_1_0_0_-1_0_0]
    /// @snippet test/test_line_segment.cpp Snippet_getIntersection2D_line_1_0_0_-1_0_0
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
