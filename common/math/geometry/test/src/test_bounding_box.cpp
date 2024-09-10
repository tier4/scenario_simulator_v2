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

#include <geometry/bounding_box.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <scenario_simulator_exception/exception.hpp>

#include "expect_eq_macros.hpp"
#include "test_utils.hpp"

TEST(BoundingBox, getPointsFromBboxDefault)
{
  geometry_msgs::msg::Pose pose;
  traffic_simulator_msgs::msg::BoundingBox bounding_box;
  std::vector<geometry_msgs::msg::Point> points = math::geometry::getPointsFromBbox(bounding_box);
  EXPECT_EQ(points.size(), size_t(4));
  EXPECT_POINT_EQ(points[0], makePoint(0.0, 0.0, 0.0));
  EXPECT_POINT_EQ(points[1], makePoint(0.0, 0.0, 0.0));
  EXPECT_POINT_EQ(points[2], makePoint(0.0, 0.0, 0.0));
  EXPECT_POINT_EQ(points[3], makePoint(0.0, 0.0, 0.0));
}

TEST(BoundingBox, getPointsFromBboxCustom)
{
  geometry_msgs::msg::Pose pose;
  traffic_simulator_msgs::msg::BoundingBox bounding_box = makeBbox(5.0, 2.0, 2.0, 1.0);
  std::vector<geometry_msgs::msg::Point> points =
    math::geometry::getPointsFromBbox(bounding_box, 1.0, 2.0, 3.0, 4.0);
  EXPECT_EQ(points.size(), size_t(4));
  EXPECT_POINT_EQ(points[0], makePoint(6.5, 3.0, 1.0));
  EXPECT_POINT_EQ(points[1], makePoint(-5.5, 3.0, 1.0));
  EXPECT_POINT_EQ(points[2], makePoint(-5.5, -2.0, 1.0));
  EXPECT_POINT_EQ(points[3], makePoint(6.5, -2.0, 1.0));
}

/**
 * @note Test obtaining polygon from bounding box with no transformation applied.
 */
TEST(BoundingBox, toPolygon2D_zeroPose)
{
  const auto pose = geometry_msgs::msg::Pose();
  const auto bounding_box = makeBbox(2.0, 2.0, 2.0);
  const auto polygon = math::geometry::toPolygon2D(pose, bounding_box);

  ASSERT_EQ(polygon.inners().size(), static_cast<std::size_t>(0));
  ASSERT_EQ(polygon.outer().size(), static_cast<std::size_t>(5));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[0], makePoint(1.0, 1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[1], makePoint(-1.0, 1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[2], makePoint(-1.0, -1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[3], makePoint(1.0, -1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[4], makePoint(1.0, 1.0));
}

/**
 * @note Test obtaining polygon from bounding box with only translation applied.
 */
TEST(BoundingBox, toPolygon2D_onlyTranslation)
{
  const auto pose = makePose(1.0, 2.0);
  const auto bounding_box = makeBbox(2.0, 2.0, 2.0);
  const auto polygon = math::geometry::toPolygon2D(pose, bounding_box);

  ASSERT_EQ(polygon.inners().size(), static_cast<std::size_t>(0));
  ASSERT_EQ(polygon.outer().size(), static_cast<std::size_t>(5));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[0], makePoint(2.0, 3.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[1], makePoint(0.0, 3.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[2], makePoint(0.0, 1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[3], makePoint(2.0, 1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[4], makePoint(2.0, 3.0));
}

/**
 * @note Test obtaining polygon from bounding box with full transformation applied (translation + rotation).
 */
TEST(BoundingBox, toPolygon2D_fullPose)
{
  const auto pose = makePose(
    1.0, 2.0, 0.0,
    math::geometry::convertEulerAngleToQuaternion(
      geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(30.0 * M_PI / 180.0)));

  const auto bounding_box = makeBbox(2.0, 2.0, 2.0);
  const auto polygon = math::geometry::toPolygon2D(pose, bounding_box);
  ASSERT_EQ(polygon.inners().size(), static_cast<std::size_t>(0));
  ASSERT_EQ(polygon.outer().size(), static_cast<std::size_t>(5));

  const double x = std::sqrt(2.0) * std::cos((30.0 + 45.0) * M_PI / 180.0);
  const double y = std::sqrt(2.0) * std::sin((30.0 + 45.0) * M_PI / 180.0);
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[0], makePoint(x + 1.0, y + 2.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[1], makePoint(-y + 1.0, x + 2.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[2], makePoint(-x + 1.0, -y + 2.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[3], makePoint(y + 1.0, -x + 2.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(polygon.outer()[4], makePoint(x + 1.0, y + 2.0));
}

TEST(BoundingBox, getPolygonDistanceWithCollision)
{
  geometry_msgs::msg::Pose pose0;
  traffic_simulator_msgs::msg::BoundingBox bbox0 = makeBbox(3.0, 3.0, 3.0);
  geometry_msgs::msg::Pose pose1;
  traffic_simulator_msgs::msg::BoundingBox bbox1 = makeBbox(1.0, 1.0, 1.0);
  EXPECT_EQ(math::geometry::getPolygonDistance(pose0, bbox0, pose1, bbox1), std::nullopt);
}

TEST(BoundingBox, getPolygonDistanceTouch)
{
  geometry_msgs::msg::Pose pose0;
  traffic_simulator_msgs::msg::BoundingBox bbox0 = makeBbox(4.0, 4.0, 4.0);
  geometry_msgs::msg::Pose pose1 = makePose(3.0, 3.0, 3.0);
  traffic_simulator_msgs::msg::BoundingBox bbox1 = makeBbox(2.0, 2.0, 2.0);
  EXPECT_EQ(math::geometry::getPolygonDistance(pose0, bbox0, pose1, bbox1), std::nullopt);
}

TEST(BoundingBox, getPolygonDistanceWithoutCollision)
{
  geometry_msgs::msg::Pose pose0;
  traffic_simulator_msgs::msg::BoundingBox bbox0 = makeBbox(3.0, 3.0, 3.0);
  geometry_msgs::msg::Pose pose1 = makePose(0.0, 5.0);
  traffic_simulator_msgs::msg::BoundingBox bbox1 = makeBbox(1.0, 1.0, 1.0);
  const auto ans = math::geometry::getPolygonDistance(pose0, bbox0, pose1, bbox1);
  EXPECT_TRUE(ans);
  EXPECT_DOUBLE_EQ(ans.value(), 3.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
