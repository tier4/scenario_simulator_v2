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

#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <iostream>
#include <sensor_simulator/sensor_simulation/primitives/primitive.hpp>
#include <string>
#include <vector>

namespace sensor_simulator
{
namespace primitives
{
Primitive::Primitive(std::string type, geometry_msgs::msg::Pose pose) : type(type), pose(pose) {}

Vertex Primitive::transform(Vertex v) const
{
  auto mat = quaternion_operation::getRotationMatrix(pose.orientation);
  Eigen::VectorXd point(3);
  point(0) = v.x;
  point(1) = v.y;
  point(2) = v.z;
  point = mat * point;
  point(0) = point(0) + pose.position.x;
  point(1) = point(1) + pose.position.y;
  point(2) = point(2) + pose.position.z;
  Vertex ret;
  ret.x = point(0);
  ret.y = point(1);
  ret.z = point(2);
  return ret;
}

std::vector<Vertex> Primitive::transform() const
{
  std::vector<Vertex> ret;
  for (auto & v : vertices_) {
    ret.emplace_back(transform(v));
  }
  return ret;
}

std::vector<Vertex> Primitive::getVertex() const { return transform(); }

std::vector<Triangle> Primitive::getTriangles() const { return triangles_; }

unsigned int Primitive::addToScene(RTCDevice device, RTCScene scene)
{
  RTCGeometry mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  const auto transformed_vertices = transform();
  Vertex * vertices = static_cast<Vertex *>(rtcSetNewGeometryBuffer(
    mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex),
    transformed_vertices.size()));
  for (size_t i = 0; i < transformed_vertices.size(); i++) {
    vertices[i] = transformed_vertices[i];
  }
  Triangle * triangles = static_cast<Triangle *>(rtcSetNewGeometryBuffer(
    mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), triangles_.size()));
  for (size_t i = 0; i < triangles_.size(); i++) {
    triangles[i] = triangles_[i];
  }
  rtcCommitGeometry(mesh);
  unsigned int geometry_id = rtcAttachGeometry(scene, mesh);
  rtcReleaseGeometry(mesh);
  return geometry_id;
}
}  // namespace primitives
}  // namespace sensor_simulator
