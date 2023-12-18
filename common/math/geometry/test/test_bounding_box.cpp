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
#include <quaternion_operation/quaternion_operation.h>

#include <geometry/bounding_box.hpp>
#include <scenario_simulator_exception/exception.hpp>

#include "expect_eq_macros.hpp"
#include "test_utils.hpp"

TEST(BoundingBox, getPointsFromBboxDefault)
{
  geometry_msgs::msg::Pose pose;
  traffic_simulator_msgs::msg::BoundingBox bbox;
  std::vector<geometry_msgs::msg::Point> points = math::geometry::getPointsFromBbox(bbox);
  EXPECT_EQ(points.size(), size_t(4));
  EXPECT_POINT_EQ(points[0], makePoint(0.0, 0.0, 0.0));
  EXPECT_POINT_EQ(points[1], makePoint(0.0, 0.0, 0.0));
  EXPECT_POINT_EQ(points[2], makePoint(0.0, 0.0, 0.0));
  EXPECT_POINT_EQ(points[3], makePoint(0.0, 0.0, 0.0));
}

TEST(BoundingBox, getPointsFromBboxCustom)
{
  geometry_msgs::msg::Pose pose;
  traffic_simulator_msgs::msg::BoundingBox bbox = makeBbox(5.0, 2.0, 2.0, 1.0);
  std::vector<geometry_msgs::msg::Point> points =
    math::geometry::getPointsFromBbox(bbox, 1.0, 2.0, 3.0, 4.0);
  EXPECT_EQ(points.size(), size_t(4));
  EXPECT_POINT_EQ(points[0], makePoint(6.5, 3.0, 1.0));
  EXPECT_POINT_EQ(points[1], makePoint(-5.5, 3.0, 1.0));
  EXPECT_POINT_EQ(points[2], makePoint(-5.5, -2.0, 1.0));
  EXPECT_POINT_EQ(points[3], makePoint(6.5, -2.0, 1.0));
}

TEST(BoundingBox, get2DPolygonZeroPose)
{
  geometry_msgs::msg::Pose pose;
  traffic_simulator_msgs::msg::BoundingBox bbox = makeBbox(2.0, 2.0, 2.0);
  boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> poly =
    math::geometry::get2DPolygon(pose, bbox);
  EXPECT_EQ(poly.inners().size(), size_t(0));
  EXPECT_EQ(poly.outer().size(), size_t(5));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[0], makePoint(1.0, 1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[1], makePoint(-1.0, 1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[2], makePoint(-1.0, -1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[3], makePoint(1.0, -1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[4], makePoint(1.0, 1.0));
}

TEST(BoundingBox, get2DPolygonOnlyTranslation)
{
  geometry_msgs::msg::Pose pose = makePose(1.0, 2.0);
  traffic_simulator_msgs::msg::BoundingBox bbox = makeBbox(2.0, 2.0, 2.0);
  boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> poly =
    math::geometry::get2DPolygon(pose, bbox);
  EXPECT_EQ(poly.inners().size(), size_t(0));
  EXPECT_EQ(poly.outer().size(), size_t(5));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[0], makePoint(2.0, 3.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[1], makePoint(0.0, 3.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[2], makePoint(0.0, 1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[3], makePoint(2.0, 1.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[4], makePoint(2.0, 3.0));
}

TEST(BoundingBox, get2DPolygonFullPose)
{
  geometry_msgs::msg::Pose pose = makePose(1.0, 2.0);
  pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(
    geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(30.0 * M_PI / 180.0));
  traffic_simulator_msgs::msg::BoundingBox bbox = makeBbox(2.0, 2.0, 2.0);
  boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> poly =
    math::geometry::get2DPolygon(pose, bbox);
  EXPECT_EQ(poly.inners().size(), size_t(0));
  EXPECT_EQ(poly.outer().size(), size_t(5));
  const double x = std::sqrt(2.0) * std::cos((30.0 + 45.0) * M_PI / 180.0);
  const double y = std::sqrt(2.0) * std::sin((30.0 + 45.0) * M_PI / 180.0);
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[0], makePoint(x + 1.0, y + 2.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[1], makePoint(-y + 1.0, x + 2.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[2], makePoint(-x + 1.0, -y + 2.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[3], makePoint(y + 1.0, -x + 2.0));
  EXPECT_BOOST_POINT_2D_AND_POINT_EQ(poly.outer()[4], makePoint(x + 1.0, y + 2.0));
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
