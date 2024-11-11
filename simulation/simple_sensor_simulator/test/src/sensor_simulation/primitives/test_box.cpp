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

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <vector>

#include "../../utils/expect_eq_macros.hpp"

using namespace simple_sensor_simulator;
using namespace simple_sensor_simulator::primitives;

/**
 * @note Test initialization correctness. The goal is to test whether the vertices and indices are
 * initialized to create a box shape (can use get2DConvexHull or other specially created const
 * reference accessors).
 */
TEST(BoxTest, Box)
{
  const std::vector<Vertex> expected_vertices = {
    {-0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, 0.5f}, {-0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, 0.5f},
    {0.5f, -0.5f, -0.5f},  {0.5f, -0.5f, 0.5f},  {0.5f, 0.5f, -0.5f},  {0.5f, 0.5f, 0.5f}};

  // Indices
  const std::vector<Triangle> expected_triangles = {{0, 1, 2}, {1, 3, 2}, {4, 6, 5}, {5, 6, 7},
                                                    {0, 4, 1}, {1, 4, 5}, {2, 3, 6}, {3, 7, 6},
                                                    {0, 2, 4}, {2, 6, 4}, {1, 5, 3}, {3, 5, 7}};

  const auto pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(0.0).y(0.0).z(0.0))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0));

  const Box box(1.0f, 1.0f, 1.0f, pose);

  const auto vertices = box.getVertex();
  const auto triangles = box.getTriangles();

  ASSERT_EQ(triangles.size(), expected_triangles.size());
  ASSERT_EQ(vertices.size(), expected_vertices.size());

  for (size_t i = 0; i < triangles.size(); ++i) {
    EXPECT_TRIANGLE_EQ(triangles[i], expected_triangles[i]);
  }

  for (size_t i = 0; i < vertices.size(); ++i) {
    EXPECT_VERTEX_EQ(vertices[i], expected_vertices[i]);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
