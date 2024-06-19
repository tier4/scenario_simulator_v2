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

#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <geometry/polygon/polygon.hpp>
#include <geometry/transform.hpp>
#include <iostream>
#include <optional>
#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
Vertex toVertex(const geometry_msgs::msg::Point & point)
{
  Vertex v;
  v.x = point.x;
  v.y = point.y;
  v.z = point.z;
  return v;
}

geometry_msgs::msg::Point toPoint(const Vertex & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x;
  p.y = v.y;
  p.z = v.z;
  return p;
}

std::vector<Vertex> toVertex(const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<Vertex> ret;
  std::transform(
    points.begin(), points.end(), std::back_inserter(ret),
    [](const geometry_msgs::msg::Point & point) { return toVertex(point); });
  return ret;
}

std::vector<geometry_msgs::msg::Point> toPoints(const std::vector<Vertex> & v)
{
  std::vector<geometry_msgs::msg::Point> ret;
  std::transform(v.begin(), v.end(), std::back_inserter(ret), [](const Vertex & vertex) {
    return toPoint(vertex);
  });
  return ret;
}

namespace primitives
{
Primitive::Primitive(std::string type, const geometry_msgs::msg::Pose & pose)
: type(type), pose(pose)
{
}

Vertex Primitive::transform(const Vertex & v) const
{
  return toVertex(math::geometry::transformPoint(pose, toPoint(v)));
}

Vertex Primitive::transform(const Vertex & v, const geometry_msgs::msg::Pose & sensor_pose) const
{
  return toVertex(math::geometry::transformPoint(pose, sensor_pose, toPoint(v)));
}

std::vector<Vertex> Primitive::transform() const
{
  std::vector<Vertex> ret;
  for (auto & v : vertices_) {
    ret.emplace_back(transform(v));
  }
  return ret;
}

std::vector<Vertex> Primitive::transform(const geometry_msgs::msg::Pose & sensor_pose) const
{
  std::vector<Vertex> ret;
  for (auto & v : vertices_) {
    ret.emplace_back(transform(v, sensor_pose));
  }
  return ret;
}

std::vector<Vertex> Primitive::getVertex() const { return transform(); }

std::vector<Triangle> Primitive::getTriangles() const { return triangles_; }

std::vector<geometry_msgs::msg::Point> Primitive::get2DConvexHull(
  const geometry_msgs::msg::Pose & sensor_pose) const
{
  return math::geometry::get2DConvexHull(toPoints(transform(sensor_pose)));
}

std::vector<geometry_msgs::msg::Point> Primitive::get2DConvexHull() const
{
  return math::geometry::get2DConvexHull(toPoints(transform()));
}

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
  // enable raycasting
  rtcSetGeometryMask(mesh, 0b11111111'11111111'11111111'11111111);
  rtcCommitGeometry(mesh);
  unsigned int geometry_id = rtcAttachGeometry(scene, mesh);
  rtcReleaseGeometry(mesh);
  return geometry_id;
}

std::optional<double> Primitive::getMax(const math::geometry::Axis & axis) const
{
  if (vertices_.empty()) {
    return std::nullopt;
  }
  return math::geometry::getMaxValue(toPoints(transform()), axis);
}

std::optional<double> Primitive::getMin(const math::geometry::Axis & axis) const
{
  if (vertices_.empty()) {
    return std::nullopt;
  }
  return math::geometry::getMinValue(toPoints(transform()), axis);
}

std::optional<double> Primitive::getMax(
  const math::geometry::Axis & axis, const geometry_msgs::msg::Pose & sensor_pose) const
{
  if (vertices_.empty()) {
    return std::nullopt;
  }
  return math::geometry::getMaxValue(toPoints(transform(sensor_pose)), axis);
}

std::optional<double> Primitive::getMin(
  const math::geometry::Axis & axis, const geometry_msgs::msg::Pose & sensor_pose) const
{
  if (vertices_.empty()) {
    return std::nullopt;
  }
  return math::geometry::getMinValue(toPoints(transform(sensor_pose)), axis);
}
}  // namespace primitives
}  // namespace simple_sensor_simulator
