// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <iostream>
#include <simple_sensor_simulator/sensor_simulation/primitives/box.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
namespace primitives
{
Box::Box(float depth, float width, float height, const geometry_msgs::msg::Pose & pose)
: Primitive("Box", pose), depth(depth), width(width), height(height)
{
  vertices_ = std::vector<Vertex>(8);

  vertices_[0].x = -0.5 * depth;
  vertices_[0].y = -0.5 * width;
  vertices_[0].z = -0.5 * height;

  vertices_[1].x = -0.5 * depth;
  vertices_[1].y = -0.5 * width;
  vertices_[1].z = +0.5 * height;

  vertices_[2].x = -0.5 * depth;
  vertices_[2].y = +0.5 * width;
  vertices_[2].z = -0.5 * height;

  vertices_[3].x = -0.5 * depth;
  vertices_[3].y = +0.5 * width;
  vertices_[3].z = +0.5 * height;

  vertices_[4].x = +0.5 * depth;
  vertices_[4].y = -0.5 * width;
  vertices_[4].z = -0.5 * height;

  vertices_[5].x = +0.5 * depth;
  vertices_[5].y = -0.5 * width;
  vertices_[5].z = +0.5 * height;

  vertices_[6].x = +0.5 * depth;
  vertices_[6].y = +0.5 * width;
  vertices_[6].z = -0.5 * height;

  vertices_[7].x = +0.5 * depth;
  vertices_[7].y = +0.5 * width;
  vertices_[7].z = +0.5 * height;

  triangles_ = std::vector<Triangle>(12);

  triangles_[0].v0 = 0;
  triangles_[0].v1 = 1;
  triangles_[0].v2 = 2;

  triangles_[1].v0 = 1;
  triangles_[1].v1 = 3;
  triangles_[1].v2 = 2;

  triangles_[2].v0 = 4;
  triangles_[2].v1 = 6;
  triangles_[2].v2 = 5;

  triangles_[3].v0 = 5;
  triangles_[3].v1 = 6;
  triangles_[3].v2 = 7;

  triangles_[4].v0 = 0;
  triangles_[4].v1 = 4;
  triangles_[4].v2 = 1;

  triangles_[5].v0 = 1;
  triangles_[5].v1 = 4;
  triangles_[5].v2 = 5;

  triangles_[6].v0 = 2;
  triangles_[6].v1 = 3;
  triangles_[6].v2 = 6;

  triangles_[7].v0 = 3;
  triangles_[7].v1 = 7;
  triangles_[7].v2 = 6;

  triangles_[8].v0 = 0;
  triangles_[8].v1 = 2;
  triangles_[8].v2 = 4;

  triangles_[9].v0 = 2;
  triangles_[9].v1 = 6;
  triangles_[9].v2 = 4;

  triangles_[10].v0 = 1;
  triangles_[10].v1 = 5;
  triangles_[10].v2 = 3;

  triangles_[11].v0 = 3;
  triangles_[11].v1 = 5;
  triangles_[11].v2 = 7;

  // transform();
}
}  // namespace primitives
}  // namespace simple_sensor_simulator
