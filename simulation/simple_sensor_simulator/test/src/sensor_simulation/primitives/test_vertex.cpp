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

#include <algorithm>
#include <geometry_msgs/msg/point.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>
#include <vector>

#include "../../utils/expect_eq_macros.hpp"

using namespace simple_sensor_simulator;

/**
 * @note Test basic functionality. Test to vertex conversion correctness with a sample point.
 */
TEST(VertexTest, toVertex_onePoint)
{
  const auto point = geometry_msgs::build<geometry_msgs::msg::Point>().x(1.0).y(2.0).z(3.0);

  const Vertex vertex = toVertex(point);

  EXPECT_VERTEX_AND_POINT_EQ(vertex, point);
}

/**
 * @note Test basic functionality. Test to vertex conversion correctness with a vector containing
 * multiple sample points.
 */
TEST(VertexTest, toVertex_manyPoints)
{
  const std::vector<geometry_msgs::msg::Point> points = {
    geometry_msgs::build<geometry_msgs::msg::Point>().x(1.0).y(2.0).z(3.0),
    geometry_msgs::build<geometry_msgs::msg::Point>().x(4.0).y(5.0).z(6.0),
    geometry_msgs::build<geometry_msgs::msg::Point>().x(7.0).y(8.0).z(9.0)};

  const std::vector<Vertex> vertices = toVertex(points);

  ASSERT_EQ(vertices.size(), points.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    EXPECT_VERTEX_AND_POINT_EQ(vertices[i], points[i]);
  }
}

/**
 * @note Test function behavior when an empty vector is passed. The goal is to get an empty vector.
 */
TEST(VertexTest, toVertex_empty)
{
  const std::vector<Vertex> vertices = toVertex(std::vector<geometry_msgs::msg::Point>{});

  EXPECT_TRUE(vertices.empty());
}

/**
 * @note Test basic functionality. Test to point conversion correctness with a sample vertex.
 */
TEST(VertexTest, toPoint_oneVertex)
{
  const Vertex vertex{1.0f, 2.0f, 3.0f};

  const geometry_msgs::msg::Point point = toPoint(vertex);

  EXPECT_VERTEX_AND_POINT_EQ(point, vertex);
}

/**
 * @note Test basic functionality. Test to point conversion correctness with a vector containing
 * multiple sample vertices.
 */
TEST(VertexTest, toPoints_manyVertices)
{
  const std::vector<Vertex> vertices = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 9.0f}};

  const std::vector<geometry_msgs::msg::Point> points = toPoints(vertices);

  ASSERT_EQ(points.size(), vertices.size());
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_VERTEX_AND_POINT_EQ(points[i], vertices[i]);
  }
}

/**
 * @note Test function behavior when an empty vector is passed. The goal is to get an empty vector.
 */
TEST(VertexTest, toPoints_empty)
{
  const std::vector<geometry_msgs::msg::Point> points = toPoints(std::vector<Vertex>{});

  EXPECT_TRUE(points.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
