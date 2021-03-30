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

#include <simulation_api/math/collision.hpp>

#include <quaternion_operation/quaternion_operation.h>

#include <boost/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>

#include <vector>
#include <iostream>

namespace simulation_api
{
namespace math
{
bool checkCollision2D(
  geometry_msgs::msg::Pose pose0, openscenario_msgs::msg::BoundingBox bbox0,
  geometry_msgs::msg::Pose pose1, openscenario_msgs::msg::BoundingBox bbox1)
{
  double z_diff_pose =
    std::fabs(
    (pose0.position.z + bbox0.center.z) -
    (pose1.position.z + bbox1.center.z));
  if (z_diff_pose > (std::fabs(bbox0.dimensions.z + bbox1.dimensions.z) * 0.5) ) {
    return false;
  }
  auto points0 = transformPoints(pose0, getPointsFromBbox(bbox0));
  auto points1 = transformPoints(pose1, getPointsFromBbox(bbox1));
  namespace bg = boost::geometry;
  typedef bg::model::d2::point_xy<double> bg_point;
  bg::model::polygon<bg_point> poly0;
  poly0.outer().push_back(bg_point(points0[0].x, points0[0].y));
  poly0.outer().push_back(bg_point(points0[1].x, points0[1].y));
  poly0.outer().push_back(bg_point(points0[2].x, points0[2].y));
  poly0.outer().push_back(bg_point(points0[3].x, points0[3].y));
  poly0.outer().push_back(bg_point(points0[0].x, points0[0].y));
  bg::model::polygon<bg_point> poly1;
  poly1.outer().push_back(bg_point(points1[0].x, points1[0].y));
  poly1.outer().push_back(bg_point(points1[1].x, points1[1].y));
  poly1.outer().push_back(bg_point(points1[2].x, points1[2].y));
  poly1.outer().push_back(bg_point(points1[3].x, points1[3].y));
  poly1.outer().push_back(bg_point(points1[0].x, points1[0].y));
  if (bg::intersects(poly0, poly1)) {
    return true;
  }
  if (bg::intersects(poly1, poly0)) {
    return true;
  }
  if (bg::disjoint(poly0, poly1)) {
    return false;
  }
  return true;
}
}  // namespace math
}  // namespace simulation_api
