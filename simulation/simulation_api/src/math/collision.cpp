// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <simulation_api/math/collision.hpp>

#include <quaternion_operation/quaternion_operation.h>

#include <boost/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign/list_of.hpp>

//headers in Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

#include <vector>

namespace simulation_api
{
namespace math
{
bool checkCollision2D(
  geometry_msgs::msg::Pose pose0, openscenario_msgs::msg::BoundingBox bbox0,
  geometry_msgs::msg::Pose pose1, openscenario_msgs::msg::BoundingBox bbox1)
{
  double z_diff_pose =
    std::fabs((pose0.position.z + bbox0.center.z) -
      (pose1.position.z + bbox1.center.z));
  if (z_diff_pose > (std::fabs(bbox0.dimensions.z + bbox1.dimensions.z) * 0.5) ) {
    return false;
  }
  auto points0 = transformPoints(pose0, getPointsFromBbox(bbox0));
  auto points1 = transformPoints(pose1, getPointsFromBbox(bbox1));
  namespace bg = boost::geometry;
  typedef bg::model::d2::point_xy<double> bg_point;
  const bg::model::linestring<bg_point> poly0 =
    boost::assign::list_of<bg_point>(points0[0].x, points0[0].y)(points0[1].x,
      points0[1].y)(points0[2].x, points0[2].y)(points0[3].x, points0[3].y);
  const bg::model::linestring<bg_point> poly1 =
    boost::assign::list_of<bg_point>(points1[0].x, points1[0].y)(points1[1].x,
      points1[1].y)(points1[2].x, points1[2].y)(points1[3].x, points1[3].y);
  if (bg::intersects(poly0, poly1)) {
    return true;
  }
  if (bg::within(poly0, poly1) || bg::within(poly0, poly1)) {
    return true;
  }
  return false;
}

std::vector<geometry_msgs::msg::Point> getPointsFromBbox(
  openscenario_msgs::msg::BoundingBox bbox)
{
  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point p0;
  p0.x = bbox.center.x + bbox.dimensions.x * 0.5;
  p0.y = bbox.center.y + bbox.dimensions.y * 0.5;
  p0.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p0);
  geometry_msgs::msg::Point p1;
  p1.x = bbox.center.x - bbox.dimensions.x * 0.5;
  p1.y = bbox.center.y + bbox.dimensions.y * 0.5;
  p1.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p1);
  geometry_msgs::msg::Point p2;
  p2.x = bbox.center.x - bbox.dimensions.x * 0.5;
  p2.y = bbox.center.y - bbox.dimensions.y * 0.5;
  p2.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p2);
  geometry_msgs::msg::Point p3;
  p3.x = bbox.center.x + bbox.dimensions.x * 0.5;
  p3.y = bbox.center.y - bbox.dimensions.y * 0.5;
  p3.z = bbox.center.z + bbox.dimensions.z * 0.5;
  points.emplace_back(p3);
  return points;
}

std::vector<geometry_msgs::msg::Point> transformPoints(
  geometry_msgs::msg::Pose pose,
  std::vector<geometry_msgs::msg::Point> points)
{
  auto mat = quaternion_operation::getRotationMatrix(pose.orientation);
  std::vector<geometry_msgs::msg::Point> ret;
  for (const auto & point : points) {
    Eigen::VectorXd v(3);
    v(0) = point.x;
    v(1) = point.y;
    v(2) = point.z;
    v = mat * v;
    v(0) = v(0) + pose.position.x;
    v(1) = v(1) + pose.position.y;
    v(2) = v(2) + pose.position.z;
    geometry_msgs::msg::Point transformed;
    transformed.x = v(0);
    transformed.y = v(1);
    transformed.z = v(2);
    ret.emplace_back(transformed);
  }
  return ret;
}
}  // namespace math
}  // namespace simulation_api
