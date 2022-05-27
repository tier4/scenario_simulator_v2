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
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <iostream>
#include <simple_sensor_simulator/sensor_simulation/primitives/primitive.hpp>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{
namespace primitives
{
Primitive::Primitive(std::string type, const geometry_msgs::msg::Pose & pose)
: type(type), pose(pose)
{
}

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

Vertex Primitive::transform(Vertex v, const geometry_msgs::msg::Pose & sensor_pose) const
{
  auto mat = quaternion_operation::getRotationMatrix(
    quaternion_operation::getRotation(sensor_pose.orientation, pose.orientation));
  Eigen::VectorXd point(3);
  point(0) = v.x;
  point(1) = v.y;
  point(2) = v.z;
  point = mat * point;
  point(0) = point(0) + pose.position.x - sensor_pose.position.x;
  point(1) = point(1) + pose.position.y - sensor_pose.position.y;
  point(2) = point(2) + pose.position.z - sensor_pose.position.z;
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
  const auto vertex = transform(sensor_pose);
  typedef boost::geometry::model::d2::point_xy<double> boost_point;
  typedef boost::geometry::model::polygon<boost_point> boost_polygon;
  boost_polygon poly;
  for (const auto & p : vertex) {
    boost::geometry::exterior_ring(poly).push_back(boost_point(p.x, p.y));
  }
  boost_polygon hull;
  boost::geometry::convex_hull(poly, hull);
  std::vector<geometry_msgs::msg::Point> polygon;
  for (auto it = boost::begin(boost::geometry::exterior_ring(hull));
       it != boost::end(boost::geometry::exterior_ring(hull)); ++it) {
    double x = boost::geometry::get<0>(*it);
    double y = boost::geometry::get<1>(*it);
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    polygon.emplace_back(p);
  }
  return polygon;
}

std::vector<geometry_msgs::msg::Point> Primitive::get2DConvexHull() const
{
  const auto vertex = getVertex();
  typedef boost::geometry::model::d2::point_xy<double> boost_point;
  typedef boost::geometry::model::polygon<boost_point> boost_polygon;
  boost_polygon poly;
  for (const auto & p : vertex) {
    boost::geometry::exterior_ring(poly).push_back(boost_point(p.x, p.y));
  }
  boost_polygon hull;
  boost::geometry::convex_hull(poly, hull);
  std::vector<geometry_msgs::msg::Point> polygon;
  for (auto it = boost::begin(boost::geometry::exterior_ring(hull));
       it != boost::end(boost::geometry::exterior_ring(hull)); ++it) {
    double x = boost::geometry::get<0>(*it);
    double y = boost::geometry::get<1>(*it);
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    polygon.emplace_back(p);
  }
  return polygon;
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
  rtcCommitGeometry(mesh);
  unsigned int geometry_id = rtcAttachGeometry(scene, mesh);
  rtcReleaseGeometry(mesh);
  return geometry_id;
}

boost::optional<double> Primitive::getMax(const Axis & axis) const
{
  std::vector<double> values;
  if (vertices_.empty()) {
    return boost::none;
  }
  const auto transformed_vertices = transform();
  for (const auto v : transformed_vertices) {
    switch (axis) {
      case Axis::X:
        values.emplace_back(v.x);
        break;
      case Axis::Y:
        values.emplace_back(v.y);
        break;
      case Axis::Z:
        values.emplace_back(v.z);
        break;
    }
  }
  return *std::max_element(values.begin(), values.end());
}

boost::optional<double> Primitive::getMin(const Axis & axis) const
{
  std::vector<double> values;
  if (vertices_.empty()) {
    return boost::none;
  }
  const auto transformed_vertices = transform();
  for (const auto v : transformed_vertices) {
    switch (axis) {
      case Axis::X:
        values.emplace_back(v.x);
        break;
      case Axis::Y:
        values.emplace_back(v.y);
        break;
      case Axis::Z:
        values.emplace_back(v.z);
        break;
    }
  }
  return *std::min_element(values.begin(), values.end());
}

boost::optional<double> Primitive::getMax(
  const Axis & axis, const geometry_msgs::msg::Pose & sensor_pose) const
{
  std::vector<double> values;
  if (vertices_.empty()) {
    return boost::none;
  }
  const auto transformed_vertices = transform(sensor_pose);
  for (const auto v : transformed_vertices) {
    switch (axis) {
      case Axis::X:
        values.emplace_back(v.x);
        break;
      case Axis::Y:
        values.emplace_back(v.y);
        break;
      case Axis::Z:
        values.emplace_back(v.z);
        break;
    }
  }
  return *std::max_element(values.begin(), values.end());
}

boost::optional<double> Primitive::getMin(
  const Axis & axis, const geometry_msgs::msg::Pose & sensor_pose) const
{
  std::vector<double> values;
  if (vertices_.empty()) {
    return boost::none;
  }
  const auto transformed_vertices = transform(sensor_pose);
  for (const auto v : transformed_vertices) {
    switch (axis) {
      case Axis::X:
        values.emplace_back(v.x);
        break;
      case Axis::Y:
        values.emplace_back(v.y);
        break;
      case Axis::Z:
        values.emplace_back(v.z);
        break;
    }
  }
  return *std::min_element(values.begin(), values.end());
}
}  // namespace primitives
}  // namespace simple_sensor_simulator
