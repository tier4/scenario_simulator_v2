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

#ifndef SIMPLE_SENSOR_SIMULATOR__TEST__TEST_PRIMITIVE_HPP_
#define SIMPLE_SENSOR_SIMULATOR__TEST__TEST_PRIMITIVE_HPP_

#include <gtest/gtest.h>
#include <quaternion_operation/quaternion_operation.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>
#include <vector>

using namespace simple_sensor_simulator::primitives;
using namespace simple_sensor_simulator;

namespace simple_sensor_simulator
{
namespace primitives
{

class DummyPrimitive : public Primitive
{
public:
  DummyPrimitive(const std::string & type, const geometry_msgs::msg::Pose & pose)
  : Primitive(type, pose)
  {
  }

  auto setVertices(const std::vector<Vertex> & vertices) -> void { vertices_ = vertices; }
  auto setTriangles(const std::vector<Triangle> & triangles) -> void { triangles_ = triangles; }

  auto getVerticesSize() const -> int { return vertices_.size(); }
  auto getTrianglesSize() const -> int { return triangles_.size(); }

  auto getVertices() const -> const std::vector<Vertex> & { return vertices_; }
  auto getTriangles() const -> const std::vector<Triangle> & { return triangles_; }
};

}  // namespace primitives
}  // namespace simple_sensor_simulator

class PrimitiveTest : public ::testing::Test
{
protected:
  PrimitiveTest()
  {
    pose_ = geometry_msgs::build<geometry_msgs::msg::Pose>()
              .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(1.0).y(2.0).z(0.0))
              .orientation(
                geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0));

    primitive_ = std::make_unique<DummyPrimitive>("DummyPrimitive", pose_);

    primitive_->setVertices(
      {{-1.0f, -1.0f, -1.0f},
       {1.0f, -1.0f, -1.0f},
       {-1.0f, 1.0f, -1.0f},
       {1.0f, 1.0f, -1.0f},
       {-1.0f, -1.0f, 1.0f},
       {1.0f, -1.0f, 1.0f},
       {-1.0f, 1.0f, 1.0f},
       {1.0f, 1.0f, 1.0f}});
    primitive_->setTriangles(
      {{0, 1, 2},
       {1, 3, 2},
       {4, 5, 6},
       {5, 7, 6},
       {0, 1, 4},
       {1, 5, 4},
       {2, 3, 6},
       {3, 7, 6},
       {0, 2, 4},
       {2, 6, 4},
       {1, 3, 5},
       {3, 7, 5}});
  }

  geometry_msgs::msg::Pose pose_;
  std::unique_ptr<DummyPrimitive> primitive_;
};

#endif  // SIMPLE_SENSOR_SIMULATOR__TEST__TEST_PRIMITIVE_HPP_
