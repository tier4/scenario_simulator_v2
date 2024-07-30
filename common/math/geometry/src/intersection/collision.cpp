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

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <geometry/bounding_box.hpp>
#include <geometry/intersection/collision.hpp>
#include <vector>

namespace math
{
namespace geometry
{
bool checkCollision2D(
  geometry_msgs::msg::Pose pose0, traffic_simulator_msgs::msg::BoundingBox bbox0,
  geometry_msgs::msg::Pose pose1, traffic_simulator_msgs::msg::BoundingBox bbox1)
{
  double z_diff_pose =
    std::abs((pose0.position.z + bbox0.center.z) - (pose1.position.z + bbox1.center.z));
  if (z_diff_pose > (std::abs(bbox0.dimensions.z + bbox1.dimensions.z) * 0.5)) {
    return false;
  }
  namespace bg = boost::geometry;
  typedef bg::model::d2::point_xy<double> bg_point;
  const bg::model::polygon<bg_point> poly0 = math::geometry::toPolygon2D(pose0, bbox0);
  const bg::model::polygon<bg_point> poly1 = math::geometry::toPolygon2D(pose1, bbox1);
  if (bg::intersects(poly0, poly1)) {
    return true;
  }
  if (bg::disjoint(poly0, poly1)) {
    return false;
  }
  return true;
}

bool contains(
  const std::vector<geometry_msgs::msg::Point> & polygon, const geometry_msgs::msg::Point & point)
{
  typedef boost::geometry::model::d2::point_xy<double> boost_point;
  typedef boost::geometry::model::polygon<boost_point> boost_polygon;
  boost_polygon poly;
  for (const auto & p : polygon) {
    boost::geometry::exterior_ring(poly).push_back(boost_point(p.x, p.y));
  }
  return boost::geometry::within(boost_point(point.x, point.y), poly);
}
}  // namespace geometry
}  // namespace math
