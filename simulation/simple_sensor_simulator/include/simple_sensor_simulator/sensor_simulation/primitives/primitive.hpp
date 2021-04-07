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

#ifndef SENSOR_SIMULATOR__SENSOR_SIMULATION__PRIMITIVES__PRIMITIVE_HPP_
#define SENSOR_SIMULATOR__SENSOR_SIMULATION__PRIMITIVES__PRIMITIVE_HPP_

#include <embree3/rtcore.h>

#include <algorithm>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
struct Vertex
{
  float x;
  float y;
  float z;
};

struct Triangle
{
  unsigned int v0;
  unsigned int v1;
  unsigned int v2;
};

namespace primitives
{
class Primitive
{
public:
  Primitive(std::string type, geometry_msgs::msg::Pose pose);
  virtual ~Primitive() = default;
  const std::string type;
  const geometry_msgs::msg::Pose pose;
  unsigned int addToScene(RTCDevice device, RTCScene scene);
  std::vector<Vertex> getVertex() const;
  std::vector<Triangle> getTriangles() const;

protected:
  std::vector<Vertex> transform() const;
  std::vector<Vertex> vertices_;
  std::vector<Triangle> triangles_;

private:
  Vertex transform(Vertex v) const;
};
}  // namespace primitives
}  // namespace simple_sensor_simulator

#endif  // SENSOR_SIMULATOR__SENSOR_SIMULATION__PRIMITIVES__PRIMITIVE_HPP_
