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

#ifndef GEOMETRY__BOUNDING_BOX_HPP_
#define GEOMETRY__BOUNDING_BOX_HPP_

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/numeric/conversion/bounds.hpp>
#include <geometry/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <vector>

namespace math
{
namespace geometry
{

typedef boost::geometry::model::d2::point_xy<double> boost_point;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;

struct DistancesFromCenterToEdge
{
  double front;
  double rear;
  double right;
  double left;
  double up;
  double down;
};

std::optional<double> getPolygonDistance(
  const geometry_msgs::msg::Pose & pose0, const traffic_simulator_msgs::msg::BoundingBox & bbox0,
  const geometry_msgs::msg::Pose & pose1, const traffic_simulator_msgs::msg::BoundingBox & bbox1);
std::optional<std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose>> getClosestPoses(
  const geometry_msgs::msg::Pose & pose0, const traffic_simulator_msgs::msg::BoundingBox & bbox0,
  const geometry_msgs::msg::Pose & pose1, const traffic_simulator_msgs::msg::BoundingBox & bbox1);
boost_point pointToSegmentProjection(
  const boost_point & p, const boost_point & p1, const boost_point & p2);
boost_point toBoostPoint(const geometry_msgs::msg::Point & point);
boost_polygon toBoostPolygon(const std::vector<geometry_msgs::msg::Point> & points);
geometry_msgs::msg::Pose toPose(const boost_point & point);
geometry_msgs::msg::Pose subtractPoses(
  const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2);
auto toPolygon2D(const traffic_simulator_msgs::msg::BoundingBox & bounding_box)
  -> std::vector<geometry_msgs::msg::Point>;
auto toPolygon2D(
  const geometry_msgs::msg::Pose & pose,
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box) -> const boost_polygon;
std::vector<geometry_msgs::msg::Point> getPointsFromBbox(
  traffic_simulator_msgs::msg::BoundingBox bounding_box, double width_extension_right = 0.0,
  double width_extension_left = 0.0, double length_extension_front = 0.0,
  double length_extension_rear = 0.0);
DistancesFromCenterToEdge getDistancesFromCenterToEdge(
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box);

}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__BOUNDING_BOX_HPP_
