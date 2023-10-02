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

TEST(BoundingBox, getPointsFromBboxDefault)
{
  geometry_msgs::msg::Pose pose;
  traffic_simulator_msgs::msg::BoundingBox bbox;
  std::vector<geometry_msgs::msg::Point> points = math::geometry::getPointsFromBbox(bbox);
  ASSERT_EQ(points.size(), size_t(4));
  ASSERT_DOUBLE_EQ(points[0].x, 0.0);
  ASSERT_DOUBLE_EQ(points[0].y, 0.0);
  ASSERT_DOUBLE_EQ(points[0].z, 0.0);
  ASSERT_DOUBLE_EQ(points[1].x, 0.0);
  ASSERT_DOUBLE_EQ(points[1].y, 0.0);
  ASSERT_DOUBLE_EQ(points[1].z, 0.0);
  ASSERT_DOUBLE_EQ(points[2].x, 0.0);
  ASSERT_DOUBLE_EQ(points[2].y, 0.0);
  ASSERT_DOUBLE_EQ(points[2].z, 0.0);
  ASSERT_DOUBLE_EQ(points[3].x, 0.0);
  ASSERT_DOUBLE_EQ(points[3].y, 0.0);
  ASSERT_DOUBLE_EQ(points[3].z, 0.0);
}

TEST(BoundingBox, getPointsFromBboxCustom)
{
  geometry_msgs::msg::Pose pose;
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.dimensions.x = 5.0;
  bbox.dimensions.y = 2.0;
  bbox.dimensions.z = 2.0;
  bbox.center.x = 1.0;
  std::vector<geometry_msgs::msg::Point> points =
    math::geometry::getPointsFromBbox(bbox, 1.0, 2.0, 3.0, 4.0);
  ASSERT_EQ(points.size(), size_t(4));
  ASSERT_DOUBLE_EQ(points[0].x, 6.5);
  ASSERT_DOUBLE_EQ(points[0].y, 3.0);
  ASSERT_DOUBLE_EQ(points[0].z, 1.0);
  ASSERT_DOUBLE_EQ(points[1].x, -5.5);
  ASSERT_DOUBLE_EQ(points[1].y, 3.0);
  ASSERT_DOUBLE_EQ(points[1].z, 1.0);
  ASSERT_DOUBLE_EQ(points[2].x, -5.5);
  ASSERT_DOUBLE_EQ(points[2].y, -2.0);
  ASSERT_DOUBLE_EQ(points[2].z, 1.0);
  ASSERT_DOUBLE_EQ(points[3].x, 6.5);
  ASSERT_DOUBLE_EQ(points[3].y, -2.0);
  ASSERT_DOUBLE_EQ(points[3].z, 1.0);
}

TEST(BoundingBox, get2DPolygonZeroPose)
{
  geometry_msgs::msg::Pose pose;
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.dimensions.x = 2.0;
  bbox.dimensions.y = 2.0;
  bbox.dimensions.z = 2.0;
  boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> poly =
    math::geometry::get2DPolygon(pose, bbox);
  EXPECT_TRUE(poly.inners().empty());
  EXPECT_EQ(poly.outer().size(), size_t(5));
  EXPECT_DOUBLE_EQ(poly.outer()[0].x(), 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[0].y(), 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[1].x(), -1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[1].y(), 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[2].x(), -1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[2].y(), -1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[3].x(), 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[3].y(), -1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[4].x(), 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[4].y(), 1.0);
}

TEST(BoundingBox, get2DPolygonOnlyTranslation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.dimensions.x = 2.0;
  bbox.dimensions.y = 2.0;
  bbox.dimensions.z = 2.0;
  boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> poly =
    math::geometry::get2DPolygon(pose, bbox);
  EXPECT_TRUE(poly.inners().empty());
  EXPECT_EQ(poly.outer().size(), size_t(5));
  EXPECT_DOUBLE_EQ(poly.outer()[0].x(), 2.0);
  EXPECT_DOUBLE_EQ(poly.outer()[0].y(), 3.0);
  EXPECT_DOUBLE_EQ(poly.outer()[1].x(), 0.0);
  EXPECT_DOUBLE_EQ(poly.outer()[1].y(), 3.0);
  EXPECT_DOUBLE_EQ(poly.outer()[2].x(), 0.0);
  EXPECT_DOUBLE_EQ(poly.outer()[2].y(), 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[3].x(), 2.0);
  EXPECT_DOUBLE_EQ(poly.outer()[3].y(), 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[4].x(), 2.0);
  EXPECT_DOUBLE_EQ(poly.outer()[4].y(), 3.0);
}

TEST(BoundingBox, get2DPolygonFullPose)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.orientation = quaternion_operation::convertEulerAngleToQuaternion([]() {
    geometry_msgs::msg::Vector3 vec;
    vec.x = 0.0;
    vec.y = 0.0;
    vec.z = 30.0 * M_PI / 180.0;
    return vec;
  }());
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.dimensions.x = 2.0;
  bbox.dimensions.y = 2.0;
  bbox.dimensions.z = 2.0;
  boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>> poly =
    math::geometry::get2DPolygon(pose, bbox);
  EXPECT_TRUE(poly.inners().empty());
  EXPECT_EQ(poly.outer().size(), size_t(5));
  const double x = std::sqrt(2.0) * std::cos((30.0 + 45.0) * M_PI / 180.0);
  const double y = std::sqrt(2.0) * std::sin((30.0 + 45.0) * M_PI / 180.0);
  EXPECT_DOUBLE_EQ(poly.outer()[0].x(), x + 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[0].y(), y + 2.0);
  EXPECT_DOUBLE_EQ(poly.outer()[1].x(), -y + 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[1].y(), x + 2.0);
  EXPECT_DOUBLE_EQ(poly.outer()[2].x(), -x + 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[2].y(), -y + 2.0);
  EXPECT_DOUBLE_EQ(poly.outer()[3].x(), y + 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[3].y(), -x + 2.0);
  EXPECT_DOUBLE_EQ(poly.outer()[4].x(), x + 1.0);
  EXPECT_DOUBLE_EQ(poly.outer()[4].y(), y + 2.0);
}

TEST(BoundingBox, getPolygonDistanceWithCollision)
{
  geometry_msgs::msg::Pose pose0;
  traffic_simulator_msgs::msg::BoundingBox bbox0;
  bbox0.dimensions.x = 3;
  bbox0.dimensions.y = 3;
  bbox0.dimensions.z = 3;
  geometry_msgs::msg::Pose pose1;
  traffic_simulator_msgs::msg::BoundingBox bbox1;
  bbox1.dimensions.x = 1;
  bbox1.dimensions.y = 1;
  bbox1.dimensions.z = 1;
  EXPECT_EQ(math::geometry::getPolygonDistance(pose0, bbox0, pose1, bbox1), std::nullopt);
}

TEST(BoundingBox, getPolygonDistanceTouch)
{
  geometry_msgs::msg::Pose pose0;
  traffic_simulator_msgs::msg::BoundingBox bbox0;
  bbox0.dimensions.x = 4;
  bbox0.dimensions.y = 4;
  bbox0.dimensions.z = 4;
  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 3;
  pose1.position.y = 3;
  pose1.position.z = 3;
  traffic_simulator_msgs::msg::BoundingBox bbox1;
  bbox1.dimensions.x = 2;
  bbox1.dimensions.y = 2;
  bbox1.dimensions.z = 2;
  EXPECT_EQ(math::geometry::getPolygonDistance(pose0, bbox0, pose1, bbox1), std::nullopt);
}

TEST(BoundingBox, getPolygonDistanceWithoutCollision)
{
  geometry_msgs::msg::Pose pose0;
  traffic_simulator_msgs::msg::BoundingBox bbox0;
  bbox0.dimensions.x = 3;
  bbox0.dimensions.y = 3;
  bbox0.dimensions.z = 3;
  geometry_msgs::msg::Pose pose1;
  pose1.position.y = 5;
  traffic_simulator_msgs::msg::BoundingBox bbox1;
  bbox1.dimensions.x = 1;
  bbox1.dimensions.y = 1;
  bbox1.dimensions.z = 1;
  EXPECT_TRUE(math::geometry::getPolygonDistance(pose0, bbox0, pose1, bbox1));
  EXPECT_DOUBLE_EQ(math::geometry::getPolygonDistance(pose0, bbox0, pose1, bbox1).value(), 3.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
