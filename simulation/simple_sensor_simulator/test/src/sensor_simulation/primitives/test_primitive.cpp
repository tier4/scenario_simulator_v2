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

#include "test_primitive.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include <vector>

#include "../../utils/expect_eq_macros.hpp"

/**
 * @note Test basic functionality. Test adding to scene correctness with a sample primitive.
 */
TEST_F(PrimitiveTest, addToScene_sample)
{
  const std::vector<Vertex> expected_vertices = {
    {0.0f, 1.0f, -1.0f}, {0.0f, 1.0f, 1.0f}, {0.0f, 3.0f, -1.0f}, {0.0f, 3.0f, 1.0f},
    {2.0f, 1.0f, -1.0f}, {2.0f, 1.0f, 1.0f}, {2.0f, 3.0f, -1.0f}, {2.0f, 3.0f, 1.0f}};
  const auto expected_triangles = primitive_->getTriangles();

  RTCDevice device = rtcNewDevice(nullptr);
  RTCScene scene = rtcNewScene(device);

  const unsigned int geom_id = primitive_->addToScene(device, scene);
  ASSERT_NE(geom_id, RTC_INVALID_GEOMETRY_ID);

  const RTCGeometry geom = rtcGetGeometry(scene, geom_id);
  ASSERT_NE(geom, nullptr);

  rtcCommitScene(scene);

  // Check vertices
  const Vertex * vertex_buffer =
    static_cast<const Vertex *>(rtcGetGeometryBufferData(geom, RTC_BUFFER_TYPE_VERTEX, 0));

  for (size_t i = 0; i < expected_vertices.size(); ++i) {
    EXPECT_VERTEX_EQ(vertex_buffer[i], expected_vertices[i])
  }

  // Check triangles
  const Triangle * triangle_buffer =
    static_cast<const Triangle *>(rtcGetGeometryBufferData(geom, RTC_BUFFER_TYPE_INDEX, 0));

  for (size_t i = 0; i < expected_vertices.size(); ++i) {
    EXPECT_TRIANGLE_EQ(triangle_buffer[i], expected_triangles[i]);
  }

  rtcReleaseScene(scene);
  rtcReleaseDevice(device);
}

/**
 * @note Test function behavior with vertices set to only zeros.
 */
TEST_F(PrimitiveTest, addToScene_zeros)
{
  const std::vector<Triangle> expected_triangles = {{0, 1, 2}};
  const std::vector<Vertex> expected_vertices = {{0.0f, 0.0f, 0.0f}};

  primitive_ =
    std::make_unique<Box>(0.0f, 0.0f, 0.0f, utils::makePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0));

  RTCDevice device = rtcNewDevice(nullptr);
  RTCScene scene = rtcNewScene(device);

  const unsigned int geom_id = primitive_->addToScene(device, scene);
  ASSERT_NE(geom_id, RTC_INVALID_GEOMETRY_ID);

  const RTCGeometry geom = rtcGetGeometry(scene, geom_id);
  ASSERT_NE(geom, nullptr);

  rtcCommitScene(scene);

  // Check vertices
  const Vertex * vertex_buffer =
    static_cast<const Vertex *>(rtcGetGeometryBufferData(geom, RTC_BUFFER_TYPE_VERTEX, 0));

  for (size_t i = 0; i < expected_vertices.size(); ++i) {
    EXPECT_VERTEX_EQ(vertex_buffer[i], expected_vertices[i])
  }

  // Check triangles
  const Triangle * triangle_buffer =
    static_cast<const Triangle *>(rtcGetGeometryBufferData(geom, RTC_BUFFER_TYPE_INDEX, 0));

  for (size_t i = 0; i < expected_triangles.size(); ++i) {
    EXPECT_TRIANGLE_EQ(triangle_buffer[i], expected_triangles[i]);
  }

  rtcReleaseScene(scene);
  rtcReleaseDevice(device);
}

/**
 * @note Test basic functionality. Test obtaining triangles correctness.
 */
TEST_F(PrimitiveTest, getTriangles)
{
  const std::vector<Triangle> expected_triangles = {{0, 1, 2}, {1, 3, 2}, {4, 6, 5}, {5, 6, 7},
                                                    {0, 4, 1}, {1, 4, 5}, {2, 3, 6}, {3, 7, 6},
                                                    {0, 2, 4}, {2, 6, 4}, {1, 5, 3}, {3, 5, 7}};

  const auto triangles = primitive_->getTriangles();

  ASSERT_EQ(triangles.size(), expected_triangles.size());
  for (size_t i = 0; i < triangles.size(); ++i) {
    EXPECT_TRIANGLE_EQ(triangles[i], expected_triangles[i]);
  }
}

/**
 * @note Test basic functionality. Test obtaining vertexes correctness.
 */
TEST_F(PrimitiveTest, getVertex)
{
  const std::vector<Vertex> expected_vertices = {
    {0.0f, 1.0f, -1.0f}, {0.0f, 1.0f, 1.0f}, {0.0f, 3.0f, -1.0f}, {0.0f, 3.0f, 1.0f},
    {2.0f, 1.0f, -1.0f}, {2.0f, 1.0f, 1.0f}, {2.0f, 3.0f, -1.0f}, {2.0f, 3.0f, 1.0f}};

  const auto vertices = primitive_->getVertex();

  ASSERT_EQ(vertices.size(), expected_vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    EXPECT_VERTEX_EQ(vertices[i], expected_vertices[i]);
  }
}

/**
 * @note Test basic functionality. Test conversion to a convex hull of some concave primitive.
 */
TEST_F(PrimitiveTest, get2DConvexHull_normal)
{
  const std::vector<geometry_msgs::msg::Point> expected_hull = {
    utils::makePoint(0.0, 1.0, 0.0), utils::makePoint(0.0, 3.0, 0.0),
    utils::makePoint(2.0, 3.0, 0.0), utils::makePoint(2.0, 1.0, 0.0),
    utils::makePoint(0.0, 1.0, 0.0)};

  const auto hull = primitive_->get2DConvexHull();

  EXPECT_GT(hull.size(), 0);
  ASSERT_EQ(hull.size(), expected_hull.size());
  for (size_t i = 0; i < hull.size(); ++i) {
    EXPECT_POINT_NEAR(hull[i], expected_hull[i], std::numeric_limits<double>::epsilon())
  }
}

/**
 * @note Test basic functionality. Test conversion to a convex hull of some concave primitive with
 * an additional sensor pose transformation - the goal is to test the transformation of convex hull.
 */
TEST_F(PrimitiveTest, get2DConvexHull_withTransform)
{
  const std::vector<geometry_msgs::msg::Point> expected_hull = {
    utils::makePoint(-1.0, 0.0, 0.0), utils::makePoint(-1.0, 2.0, 0.0),
    utils::makePoint(1.0, 2.0, 0.0), utils::makePoint(1.0, 0.0, 0.0),
    utils::makePoint(-1.0, 0.0, 0.0)};

  const auto sensor_pose = utils::makePose(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0);

  const auto hull = primitive_->get2DConvexHull(sensor_pose);

  EXPECT_GT(hull.size(), 0);
  ASSERT_EQ(hull.size(), expected_hull.size());
  for (size_t i = 0; i < hull.size(); ++i) {
    EXPECT_POINT_NEAR(hull[i], expected_hull[i], std::numeric_limits<double>::epsilon())
  }
}

/**
 * @note Test basic functionality. Test min value obtaining in a given axis with a sample primitive.
 */
TEST_F(PrimitiveTest, getMin)
{
  const auto min_x = primitive_->getMin(math::geometry::Axis::X);

  ASSERT_TRUE(min_x.has_value());
  EXPECT_NEAR(min_x.value(), 0.0f, std::numeric_limits<double>::epsilon());
}

/**
 * @note Test basic functionality. Test min value obtaining in a given axis with a transformation
 * with a sample primitive and non trivial sensor pose.
 */
TEST_F(PrimitiveTest, getMin_withTransform)
{
  const auto sensor_pose = utils::makePose(5.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0);

  const auto min_x = primitive_->getMin(math::geometry::Axis::X, sensor_pose);

  ASSERT_TRUE(min_x.has_value());
  EXPECT_NEAR(min_x.value(), -5.0f, std::numeric_limits<double>::epsilon());
}

/**
 * @note Test function behavior when vertices is an empty vector - the goal is to get a nullopt.
 */
TEST_F(PrimitiveTest, getMin_empty)
{
  primitive_ = std::make_unique<Primitive>("Unknown", pose_);

  const auto min_x = primitive_->getMin(math::geometry::Axis::X);

  EXPECT_FALSE(min_x.has_value());
}

/**
 * @note Test basic functionality. Test max value obtaining in a given axis with a sample primitive.
 */
TEST_F(PrimitiveTest, getMax)
{
  const auto max_x = primitive_->getMax(math::geometry::Axis::X);

  ASSERT_TRUE(max_x.has_value());
  EXPECT_NEAR(max_x.value(), 2.0f, std::numeric_limits<double>::epsilon());
}

/**
 * @note Test basic functionality. Test max value obtaining in a given axis with a transformation
 * with a sample primitive and non trivial sensor pose.
 */
TEST_F(PrimitiveTest, getMax_withTransform)
{
  const auto sensor_pose = utils::makePose(6.0, 3.0, 2.0, 0.0, 0.0, 0.0, 1.0);

  const auto max_x = primitive_->getMax(math::geometry::Axis::X, sensor_pose);

  ASSERT_TRUE(max_x.has_value());
  EXPECT_NEAR(max_x.value(), -4.0f, std::numeric_limits<double>::epsilon());
}

/**
 * @note Test function behavior when vertices is an empty vector - the goal is to get a nullopt.
 */
TEST_F(PrimitiveTest, getMax_empty)
{
  primitive_ = std::make_unique<Primitive>("Unknown", pose_);

  const auto max_x = primitive_->getMax(math::geometry::Axis::X);

  EXPECT_FALSE(max_x.has_value());
}
